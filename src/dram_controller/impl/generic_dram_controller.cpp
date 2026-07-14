#include "dram_controller/controller.h"
#include "dram_controller/Controller_MDGE.h"
#include "memory_system/memory_system.h"
#include "addr_mapper/addr_mapper.h"

#include <memory>

namespace Ramulator {

class GenericDRAMController : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, GenericDRAMController, "Generic", "A generic DRAM controller.");
  private:
    std::deque<Request> pending;          // A queue for read requests that are about to finish (callback after RL)

    ReqBuffer m_active_buffer;            // Buffer for requests being served. This has the highest priority 
    ReqBuffer m_priority_buffer;          // Buffer for high-priority requests (e.g., maintenance like refresh).
    ReqBuffer m_read_buffer;              // Read request buffer
    ReqBuffer m_write_buffer;             // Write request buffer

    int m_bank_addr_idx = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    bool  m_is_write_mode = false;

    size_t s_row_hits = 0;
    size_t s_row_misses = 0;
    size_t s_row_conflicts = 0;
    size_t s_read_row_hits = 0;
    size_t s_read_row_misses = 0;
    size_t s_read_row_conflicts = 0;
    size_t s_write_row_hits = 0;
    size_t s_write_row_misses = 0;
    size_t s_write_row_conflicts = 0;

    size_t m_num_cores = 0;
    std::vector<size_t> s_read_row_hits_per_core;
    std::vector<size_t> s_read_row_misses_per_core;
    std::vector<size_t> s_read_row_conflicts_per_core;

    size_t s_num_read_reqs = 0;
    size_t s_num_write_reqs = 0;
    size_t s_num_other_reqs = 0;
    size_t s_queue_len = 0;
    size_t s_read_queue_len = 0;
    size_t s_write_queue_len = 0;
    size_t s_priority_queue_len = 0;
    float s_queue_len_avg = 0;
    float s_read_queue_len_avg = 0;
    float s_write_queue_len_avg = 0;
    float s_priority_queue_len_avg = 0;

    size_t s_read_latency = 0;
    float s_avg_read_latency = 0;


  public:
    void init() override {
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);

      m_scheduler = create_child_ifce<IScheduler>();
      m_refresh = create_child_ifce<IRefreshManager>();    
      m_rowpolicy = create_child_ifce<IRowPolicy>();    

      if (m_config["plugins"]) {
        YAML::Node plugin_configs = m_config["plugins"];
        for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it) {
          m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
        }
      }
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();
      m_bank_addr_idx = m_dram->m_levels("bank");
      m_priority_buffer.max_size = 512*3 + 32;

      m_num_cores = frontend->get_num_cores();

      s_read_row_hits_per_core.resize(m_num_cores, 0);
      s_read_row_misses_per_core.resize(m_num_cores, 0);
      s_read_row_conflicts_per_core.resize(m_num_cores, 0);

      register_stat(s_row_hits).name("row_hits_{}", m_channel_id);
      register_stat(s_row_misses).name("row_misses_{}", m_channel_id);
      register_stat(s_row_conflicts).name("row_conflicts_{}", m_channel_id);
      register_stat(s_read_row_hits).name("read_row_hits_{}", m_channel_id);
      register_stat(s_read_row_misses).name("read_row_misses_{}", m_channel_id);
      register_stat(s_read_row_conflicts).name("read_row_conflicts_{}", m_channel_id);
      register_stat(s_write_row_hits).name("write_row_hits_{}", m_channel_id);
      register_stat(s_write_row_misses).name("write_row_misses_{}", m_channel_id);
      register_stat(s_write_row_conflicts).name("write_row_conflicts_{}", m_channel_id);

      for (size_t core_id = 0; core_id < m_num_cores; core_id++) {
        register_stat(s_read_row_hits_per_core[core_id]).name("read_row_hits_core_{}", core_id);
        register_stat(s_read_row_misses_per_core[core_id]).name("read_row_misses_core_{}", core_id);
        register_stat(s_read_row_conflicts_per_core[core_id]).name("read_row_conflicts_core_{}", core_id);
      }

      register_stat(s_num_read_reqs).name("num_read_reqs_{}", m_channel_id);
      register_stat(s_num_write_reqs).name("num_write_reqs_{}", m_channel_id);
      register_stat(s_num_other_reqs).name("num_other_reqs_{}", m_channel_id);
      register_stat(s_queue_len).name("queue_len_{}", m_channel_id);
      register_stat(s_read_queue_len).name("read_queue_len_{}", m_channel_id);
      register_stat(s_write_queue_len).name("write_queue_len_{}", m_channel_id);
      register_stat(s_priority_queue_len).name("priority_queue_len_{}", m_channel_id);
      register_stat(s_queue_len_avg).name("queue_len_avg_{}", m_channel_id);
      register_stat(s_read_queue_len_avg).name("read_queue_len_avg_{}", m_channel_id);
      register_stat(s_write_queue_len_avg).name("write_queue_len_avg_{}", m_channel_id);
      register_stat(s_priority_queue_len_avg).name("priority_queue_len_avg_{}", m_channel_id);

      register_stat(s_read_latency).name("read_latency_{}", m_channel_id);
      register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);
    };

    bool send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      switch (req.type_id) {
        case Request::Type::Read: {
          s_num_read_reqs++;
          break;
        }
        case Request::Type::Write: {
          s_num_write_reqs++;
          break;
        }
        default: {
          s_num_other_reqs++;
          break;
        }
      }

      // Forward existing write requests to incoming read requests
      if (req.type_id == Request::Type::Read) {
        auto compare_addr = [req](const Request& wreq) {
          return wreq.addr == req.addr;
        };
        if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(), compare_addr) != m_write_buffer.end()) {
          // The request will depart at the next cycle
          req.depart = m_clk + 1;
          pending.push_back(req);
          return true;
        }
      }

      // Else, enqueue them to corresponding buffer based on request type id
      bool is_success = false;
      req.arrive = m_clk;
      if        (req.type_id == Request::Type::Read) {
        is_success = m_read_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::Write) {
        is_success = m_write_buffer.enqueue(req);
      } else {
        throw std::runtime_error("Invalid request type!");
      }
      if (!is_success) {
        // We could not enqueue the request
        req.arrive = -1;
        return false;
      }

      return true;
    };

    bool priority_send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      bool is_success = false;
      is_success = m_priority_buffer.enqueue(req);
      return is_success;
    }

    void tick() override {
      m_clk++;

      // Update statistics
      s_queue_len += m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() + pending.size();
      s_read_queue_len += m_read_buffer.size() + pending.size();
      s_write_queue_len += m_write_buffer.size();
      s_priority_queue_len += m_priority_buffer.size();

      // 1. Serve completed reads
      serve_completed_reads();

      m_refresh->tick();

      // 2. Try to find a request to serve.
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      bool request_found = schedule_request(req_it, buffer);

      // 2.1 Take row policy action
      m_rowpolicy->update(request_found, req_it);

      // 3. Update all plugins
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      // 4. Finally, issue the commands to serve the request
      if (request_found) {
        // If we find a real request to serve
        if (req_it->is_stat_updated == false) {
          update_request_stats(req_it);
        }
        m_dram->issue_command(req_it->command, req_it->addr_vec);

        // If we are issuing the last command, set depart clock cycle and move the request to the pending queue
        if (req_it->command == req_it->final_command) {
          if (req_it->type_id == Request::Type::Read) {
            req_it->depart = m_clk + m_dram->m_read_latency;
            pending.push_back(*req_it);
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
          }
          buffer->remove(req_it);
        } else {
          if (m_dram->m_command_meta(req_it->command).is_opening) {
            if (m_active_buffer.enqueue(*req_it)) {
              buffer->remove(req_it);
            }
          }
        }

      }

    };


  private:
    /**
     * @brief    Helper function to check if a request is hitting an open row
     * @details
     * 
     */
    bool is_row_hit(ReqBuffer::iterator& req)
    {
        return m_dram->check_rowbuffer_hit(req->final_command, req->addr_vec);
    }
    /**
     * @brief    Helper function to check if a request is opening a row
     * @details
     * 
    */
    bool is_row_open(ReqBuffer::iterator& req)
    {
        return m_dram->check_node_open(req->final_command, req->addr_vec);
    }

    /**
     * @brief    
     * @details
     * 
     */
    void update_request_stats(ReqBuffer::iterator& req)
    {
      req->is_stat_updated = true;

      if (req->type_id == Request::Type::Read) 
      {
        if (is_row_hit(req)) {
          s_read_row_hits++;
          s_row_hits++;
          if (req->source_id != -1)
            s_read_row_hits_per_core[req->source_id]++;
        } else if (is_row_open(req)) {
          s_read_row_conflicts++;
          s_row_conflicts++;
          if (req->source_id != -1)
            s_read_row_conflicts_per_core[req->source_id]++;
        } else {
          s_read_row_misses++;
          s_row_misses++;
          if (req->source_id != -1)
            s_read_row_misses_per_core[req->source_id]++;
        } 
      } 
      else if (req->type_id == Request::Type::Write) 
      {
        if (is_row_hit(req)) {
          s_write_row_hits++;
          s_row_hits++;
        } else if (is_row_open(req)) {
          s_write_row_conflicts++;
          s_row_conflicts++;
        } else {
          s_write_row_misses++;
          s_row_misses++;
        }
      }
    }

    /**
     * @brief    Helper function to serve the completed read requests
     * @details
     * This function is called at the beginning of the tick() function.
     * It checks the pending queue to see if the top request has received data from DRAM.
     * If so, it finishes this request by calling its callback and poping it from the pending queue.
     */
    void serve_completed_reads() {
      if (pending.size()) {
        // Check the first pending request
        auto& req = pending[0];
        if (req.depart <= m_clk) {
          // Request received data from dram
          if (req.depart - req.arrive > 1) {
            // Check if this requests accesses the DRAM or is being forwarded.
            // TODO add the stats back
            s_read_latency += req.depart - req.arrive;
          }

          if (req.callback) {
            // If the request comes from outside (e.g., processor), call its callback
            req.callback(req);
          }
          // Finally, remove this request from the pending queue
          pending.pop_front();
        }
      };
    };


    /**
     * @brief    Checks if we need to switch to write mode
     * 
     */
    void set_write_mode() {
      if (!m_is_write_mode) {
        if ((m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size) || m_read_buffer.size() == 0) {
          m_is_write_mode = true;
        }
      } else {
        if ((m_write_buffer.size() < m_wr_low_watermark * m_write_buffer.max_size) && m_read_buffer.size() != 0) {
          m_is_write_mode = false;
        }
      }
    };


    /**
     * @brief    Helper function to find a request to schedule from the buffers.
     * 
     */
    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      bool request_found = false;
      // 2.1    First, check the act buffer to serve requests that are already activating (avoid useless ACTs)
      if (req_it= m_scheduler->get_best_request(m_active_buffer); req_it != m_active_buffer.end()) {
        if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
          request_found = true;
          req_buffer = &m_active_buffer;
        }
      }

      // 2.2    If no requests can be scheduled from the act buffer, check the rest of the buffers
      if (!request_found) {
        // 2.2.1    We first check the priority buffer to prioritize e.g., maintenance requests
        if (m_priority_buffer.size() != 0) {
          req_buffer = &m_priority_buffer;
          req_it = m_priority_buffer.begin();
          req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
          
          request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
          if (!request_found & m_priority_buffer.size() != 0) {
            return false;
          }
        }

        // 2.2.1    If no request to be scheduled in the priority buffer, check the read and write buffers.
        if (!request_found) {
          // Query the write policy to decide which buffer to serve
          set_write_mode();
          auto& buffer = m_is_write_mode ? m_write_buffer : m_read_buffer;
          if (req_it = m_scheduler->get_best_request(buffer); req_it != buffer.end()) {
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
            req_buffer = &buffer;
          }
        }
      }

      // 2.3 If we find a request to schedule, we need to check if it will close an opened row in the active buffer.
      if (request_found) {
        if (m_dram->m_command_meta(req_it->command).is_closing) {
          auto& rowgroup = req_it->addr_vec;
          for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
            auto& _it_rowgroup = _it->addr_vec;
            bool is_matching = true;
            for (int i = 0; i < m_bank_addr_idx + 1 ; i++) {
              if (_it_rowgroup[i] != rowgroup[i] && _it_rowgroup[i] != -1 && rowgroup[i] != -1) {
                is_matching = false;
                break;
              }
            }
            if (is_matching) {
              request_found = false;
              break;
            }
          }
        }
      }

      return request_found;
    }

    void finalize() override {
      s_avg_read_latency = (float) s_read_latency / (float) s_num_read_reqs;

      s_queue_len_avg = (float) s_queue_len / (float) m_clk;
      s_read_queue_len_avg = (float) s_read_queue_len / (float) m_clk;
      s_write_queue_len_avg = (float) s_write_queue_len / (float) m_clk;
      s_priority_queue_len_avg = (float) s_priority_queue_len / (float) m_clk;

      return;
    }

};

// ===================================================================
// MDGEController — GenericDRAMController augmented with DeFrag MDGE
// four-state machine. Provides:
//   - TranslationTable address redirect (orig → sink) on send()
//   - Per-window RCR tracking in MDGE Monitor state
//   - Background DeFrag request injection (gather READs + evacuate WRITEs)
//     into the standard read/write buffers via the parent's send()
//
// Registered as the impl named "MDGE"; select via YAML:
//   Controller:
//     impl: MDGE
//     window_size: 256
//     ...
//
// Note: we don't use RAMULATOR_REGISTER_IMPLEMENTATION here because that
// macro generates an `Implementation(config, ...)` initializer list that
// requires `Implementation` to be a DIRECT base. Since MDGEController
// inherits from GenericDRAMController (which itself inherits from
// Implementation), we hand-roll the equivalent registration so the base
// ctor is correctly `GenericDRAMController(config, parent)`.
// ===================================================================
class MDGEController : public GenericDRAMController {
public:
  // Manual equivalents of the static-name fields generated by the macro.
  inline static const std::string m_ifce_name = IDRAMController::get_name();
  inline static const std::string m_name = "MDGE";
  inline static const std::string m_desc =
    "Generic DRAM controller augmented with DeFrag MDGE four-state machine.";

protected:
  // Macro-equivalent interface overrides.
  std::string get_name() const override { return m_name; }
  std::string get_desc() const override { return m_desc; }
  std::string get_ifce_name() const override { return m_ifce_name; }

public:
  // P0 Fix #1d: capture the address mapper so we can populate addr_vec
  // for background gather/evacuate requests. Without this, those
  // requests reach schedule_request() with an empty addr_vec and crash
  // (check_ready dereferences it). The foreground path is unaffected
  // because GenericDRAMSystem::send() applies the mapper before
  // dispatching to the channel controller.
  IAddrMapper* m_addr_mapper = nullptr;

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    GenericDRAMController::setup(frontend, memory_system);
    m_addr_mapper = memory_system->get_ifce<IAddrMapper>();
  }
  // Constructor: forward to GenericDRAMController(config, parent), then
  // do the macro's post-construction housekeeping (m_impl binding, param
  // group registration, init() invocation).
  MDGEController(const YAML::Node& config, Implementation* parent)
    : GenericDRAMController(config, parent) {
    IDRAMController::m_impl = this;
    m_params.set_impl_name(m_name);
    init();
  }

  static Implementation* make_MDGEController(const YAML::Node& config,
                                             Implementation* parent) {
    return new MDGEController(config, parent);
  }
  static inline bool registered = Factory::register_implementation(
    IDRAMController::get_name(), m_name, m_desc, &make_MDGEController);

  void init() override {
    // Read MDGE tunables from YAML (params are siblings of `impl:`).
    MDGEConfig cfg;
    cfg.window_size = param<int>("window_size")
                        .desc("Decode steps per MDGE monitoring window.")
                        .default_val(256);
    cfg.t_min = param<int>("t_min")
                  .desc("AdaptiveT clamp lower bound.").default_val(64);
    cfg.t_max = param<int>("t_max")
                  .desc("AdaptiveT clamp upper bound; > this disables DeFrag.")
                  .default_val(1024);
    cfg.t_init = param<int>("t_init")
                   .desc("Initial AdaptiveT threshold.").default_val(256);
    cfg.t_idle = param<int>("t_idle")
                   .desc("Cycles of decode-idle before triggering Gather.")
                   .default_val(32);
    cfg.alpha = param<float>("alpha")
                  .desc("AdaptiveT success shrink coefficient.").default_val(0.2f);
    cfg.beta = param<float>("beta")
                 .desc("AdaptiveT failure grow coefficient.").default_val(0.1f);
    cfg.k_hot = param<int>("k_hot")
                  .desc("Hot bank groups per channel (CGBC K).").default_val(2);
    cfg.num_channels = param<int>("num_channels")
                         .desc("Physical DRAM channels (Decision A: 8).")
                         .default_val(8);
    cfg.num_banks = param<int>("num_banks")
                      .desc("Banks per channel / rank.").default_val(16);
    cfg.num_attention_heads = param<int>("num_attention_heads")
                                .desc("Transformer attention heads.")
                                .default_val(28);
    int tt_mb = param<int>("tt_size_mb")
                  .desc("TranslationTable on-chip storage budget in MB.")
                  .default_val(4);
    (void)tt_mb;  // capacity wired inside Controller_MDGE::configure()

    m_mdge = std::make_unique<Controller_MDGE>();
    m_mdge->configure(cfg);
  }

  bool send(Request& req) override {
    if (m_mdge) {
      // (1) Address redirect: if MDGE previously evacuated this row, route
      //     the access to its sink address instead.
      uint64_t sink = 0;
      if (m_mdge->translation_table()->lookup(req.addr, sink)) {
        req.addr = sink;
      }
      // (2) Hand off to MDGE monitor (per-window RCR bookkeeping).
      //     head_id threading is future work (use scratchpad[0]) — for now
      //     monitor just sees the raw request address.
      m_mdge->on_receive(req);
    }
    return GenericDRAMController::send(req);
  }

  void tick() override {
    if (m_mdge) {
      m_mdge->tick((uint64_t)m_clk);
      // Drain background DeFrag requests (gather reads + evacuate writes)
      // into the parent's read/write buffers. On overflow we drop and let
      // the next tick try again — acceptable for first-pass smoke test.
      Request bg(0, (int)Request::Type::Read);
      while (m_mdge->defrag_queue()->dequeue(bg)) {
        // P0 Fix #1d: apply addr_mapper so addr_vec is populated before
        // schedule_request dereferences it. Foreground requests get this
        // for free from GenericDRAMSystem::send(); BG requests bypass
        // that path because they're injected from the controller.
        if (m_addr_mapper) m_addr_mapper->apply(bg);
        bool ok = GenericDRAMController::send(bg);
        if (!ok) break;
        // Re-prime `bg` before the next iteration: Request has no default
        // ctor, so we re-construct in-place via placement-new semantics
        // by overwriting all fields.
        bg = Request(0, (int)Request::Type::Read);
      }
    }
    GenericDRAMController::tick();
  }

private:
  std::unique_ptr<Controller_MDGE> m_mdge;
};

}   // namespace Ramulator
