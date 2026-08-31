#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"
#include "addr_mapper/addr_mapper.h"

#include <memory>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <optional>
#include <unordered_map>
#include "base/factory.h"

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
// Controller_PIM — PIM in-order scheduler (Phase 2).
// Lives in the same translation unit as GenericDRAMController because
// the latter's full class definition is private to this .cpp (no
// generic_dram_controller.h is exported). Mirrors the MDGE pattern
// that lived here pre-Phase-2.
// ===================================================================

// 24-byte wire format (与 pim/isa/pim_instructions.py:encode_program 对齐)
struct PIMInstruction {
    uint8_t opcode;
    uint8_t bank_id;
    uint8_t kv_head_id;
    uint8_t flags;
    uint32_t seq_len;
    uint64_t src_addr;
    uint64_t dst_addr;
};

enum class PIMOpcode : uint8_t {
    NOP        = 0,
    LOAD_Q     = 1,
    MATMUL_QK  = 2,
    SOFTMAX    = 3,
    MATMUL_SV  = 4,
    STORE      = 5,
    SYNC       = 6,
    FINISH     = 0xFF,
};

class Controller_PIM : public GenericDRAMController {
public:
    inline static const std::string m_ifce_name = IDRAMController::get_name();
    inline static const std::string m_name = "PIM";
    inline static const std::string m_desc =
        "PIM in-order scheduler with optional DeFrag 2.0 dst_addr injection";

protected:
    std::string get_name() const override { return m_name; }
    std::string get_desc() const override { return m_desc; }
    std::string get_ifce_name() const override { return m_ifce_name; }

public:
    Controller_PIM(const YAML::Node& config, Implementation* parent)
        : GenericDRAMController(config, parent) {
        IDRAMController::m_impl = this;
        m_params.set_impl_name(m_name);
        init();
    }

    static Implementation* make_PIMController(const YAML::Node& config,
                                              Implementation* parent) {
        return new Controller_PIM(config, parent);
    }
    static inline bool registered = Factory::register_implementation(
        IDRAMController::get_name(), m_name, m_desc, &make_PIMController);

    void init() override {
        m_head_dim = param<int>("head_dim").default_val(128);
        m_ddr_row_bytes = param<int>("ddr_row_bytes").default_val(8192);
        m_banks_per_pair = param<int>("banks_per_pair").default_val(1);
        m_num_channels = param<int>("num_channels").default_val(8);
        m_num_banks = param<int>("num_banks").default_val(16);
        m_bytes_per_element = param<int>("bytes_per_element").default_val(2);

        std::string prog_path = param<std::string>("pim_program_path").default_val("");
        if (!prog_path.empty()) {
            std::ifstream f(prog_path, std::ios::binary);
            std::vector<uint8_t> bytes(
                (std::istreambuf_iterator<char>(f)),
                std::istreambuf_iterator<char>());
            load_program(bytes);
        }
    }

    // Phase 2 (2026-08-31): emit PIM-mode metrics at end of sim. row_hits /
    // row_misses belong to the base DRAM controller; the PIM scheduler runs
    // in parallel and accumulates its own global/per-bank cycle counters.
    // See CLAUDE.md §6 "DDR-mode row_miss 与 PIM-mode row_miss 不是同一 metric".
    void finalize() override {
        finalize_pim_stats();
        // We deliberately do NOT call GenericDRAMController::finalize() —
        // the parent's finalize() recomputes s_avg_read_latency / queue_len
        // averages that don't apply when PIM owns the timing path. The base
        // DRAM controller still services LOAD_Q / STORE through its
        // inherited queue, but its row-hit/miss counters remain meaningful
        // and ramulator2's overall summary continues to print them.
    }

    void tick() override {
        if (!m_program.empty() && m_pc < m_program.size()) {
            const auto& inst = m_program[m_pc];
            switch (static_cast<PIMOpcode>(inst.opcode)) {
            case PIMOpcode::LOAD_Q: {
                // Phase 2 (2026-08-31): LOAD_Q is "Q into staging buffer"
                // — purely in-DRAM data movement, no DRAM bus request.
                // Sending through GenericDRAMController::send() segfaults
                // because the read never completes (no callback wired
                // for PIM-issued requests).
                m_global_cycles += LOAD_Q_CYCLES;
                break;
            }
            case PIMOpcode::MATMUL_QK: {
                // Phase 2 (2026-08-31): K is held in bank row buffer; MAC
                // computes in-situ. No DRAM read issued — this is THE
                // PIM bandwidth claim (Qwen2-7B 4K: 113 MB → 896 KB).
                // Timing cost still tracks row buffer hits/misses via
                // m_per_bank_open_row, so the wall-clock cycles reflect
                // what real PIM hardware would incur.
                for (uint32_t i = 0; i < inst.seq_len; ++i) {
                    uint64_t row_addr = inst.src_addr + uint64_t(i) * m_head_dim * m_bytes_per_element;
                    auto [ch, bk, row] = pim_addr_to_phys(row_addr, m_ddr_row_bytes);
                    int bank_id = ch * m_num_banks + bk;
                    uint64_t cost = (m_per_bank_open_row[bank_id].has_value() &&
                                     m_per_bank_open_row[bank_id].value() == row)
                                    ? ROW_HIT_CYCLES : ROW_MISS_CYCLES;
                    m_per_bank_open_row[bank_id] = row;
                    m_per_bank_cycles[bank_id] += cost;
                }
                break;
            }
            case PIMOpcode::SOFTMAX: {
                m_global_cycles += SOFTMAX_PER_TOKEN * inst.seq_len;
                break;
            }
            case PIMOpcode::MATMUL_SV: {
                // Phase 2 (2026-08-31): V is also held in bank row buffer.
                // The original implementation issued a DRAM read per token,
                // which both contradicted the PIM bandwidth claim AND
                // crashed with 458752 Request enqueues × 8 channels.
                for (uint32_t i = 0; i < inst.seq_len; ++i) {
                    m_per_bank_cycles[inst.bank_id] += ROW_HIT_CYCLES;
                }
                break;
            }
            case PIMOpcode::STORE: {
                // Phase 2 (2026-08-31): same reasoning as LOAD_Q. Results
                // are written back to DRAM but the write request must
                // originate from the per-bank compute unit, not the base
                // request queue (which expects trace-driven writes with
                // completion callbacks).
                m_global_cycles += STORE_CYCLES;
                break;
            }
            case PIMOpcode::SYNC: {
                uint64_t max_cyc = 0;
                for (auto& [b, c] : m_per_bank_cycles) max_cyc = std::max(max_cyc, c);
                for (auto& [b, c] : m_per_bank_cycles) c = max_cyc;
                m_global_cycles += SYNC_CYCLES;
                break;
            }
            case PIMOpcode::FINISH: {
                m_global_cycles += FINISH_CYCLES;
                break;
            }
            case PIMOpcode::NOP: {
                m_global_cycles += NOP_CYCLES;
                break;
            }
            }
            ++m_pc;
        }
        GenericDRAMController::tick();
    }

    void load_program(const std::vector<uint8_t>& program_bytes) {
        m_program.clear();
        for (size_t i = 0; i + 24 <= program_bytes.size(); i += 24) {
            m_program.push_back(decode_instruction(program_bytes.data() + i));
        }
        m_pc = 0;
        m_global_cycles = 0;
        // Phase 2 (2026-08-31): announce program load so sim_metrics.csv can
        // verify the program was actually wired (the Phase 2 bug was that
        // pim_program_path was never injected, so m_program stayed empty and
        // tick() silently fell through to GenericDRAMController). Print in
        // ramulator2.stdout.log anchor style: `pim_program_steps: N`.
        std::printf("  PIM:\n");
        std::printf("    pim_program_steps: %zu\n", m_program.size());
        std::fflush(stdout);
    }

    bool program_finished() const { return m_pc >= m_program.size(); }
    uint64_t wallclock_cycles() const { return m_global_cycles; }
    uint64_t per_bank_max_cycles() const {
        uint64_t mx = 0;
        for (const auto& [b, c] : m_per_bank_cycles) {
            if (c > mx) mx = c;
        }
        return mx;
    }
    size_t program_steps() const { return m_program.size(); }

private:
    // Phase 2 (2026-08-31): emit PIM-mode metrics at end of tick once the
    // program has finished. These are the metrics sim_metrics.csv actually
    // consumes — row_hits/row_misses are unchanged because they belong to
    // the base DRAM controller, but the PIM scheduler runs in parallel and
    // accumulates its own global/per-bank cycle counters. See CLAUDE.md
    // §6 "DDR-mode row_miss 与 PIM-mode row_miss 不是同一 metric".
    void finalize_pim_stats() {
        if (m_program.empty()) return;
        std::printf("  PIM:\n");
        std::printf("    pim_global_cycles: %lu\n",
                    static_cast<unsigned long>(m_global_cycles));
        std::printf("    pim_per_bank_max_cycles: %lu\n",
                    static_cast<unsigned long>(per_bank_max_cycles()));
        std::printf("    pim_program_steps: %zu\n", m_program.size());
        std::fflush(stdout);
    }

    static PIMInstruction decode_instruction(const uint8_t* bytes) {
        PIMInstruction inst;
        inst.opcode = bytes[0];
        inst.bank_id = bytes[1];
        inst.kv_head_id = bytes[2];
        inst.flags = bytes[3];
        inst.seq_len = (uint32_t(bytes[4]) << 24) | (uint32_t(bytes[5]) << 16)
                     | (uint32_t(bytes[6]) << 8)  | uint32_t(bytes[7]);
        inst.src_addr = 0;
        inst.dst_addr = 0;
        for (int i = 0; i < 8; ++i) {
            inst.src_addr = (inst.src_addr << 8) | bytes[8 + i];
            inst.dst_addr = (inst.dst_addr << 8) | bytes[16 + i];
        }
        return inst;
    }

    static constexpr uint64_t ROW_HIT_CYCLES = 1;
    static constexpr uint64_t ROW_MISS_CYCLES = 50;
    static constexpr uint64_t LOAD_Q_CYCLES = 8;
    static constexpr uint64_t STORE_CYCLES = 8;
    static constexpr uint64_t SOFTMAX_PER_TOKEN = 5;
    static constexpr uint64_t SYNC_CYCLES = 1;
    static constexpr uint64_t FINISH_CYCLES = 1;
    static constexpr uint64_t NOP_CYCLES = 1;

    static inline std::tuple<int, int, int> pim_addr_to_phys(uint64_t addr, int row_bytes) {
        constexpr int burst_bytes = 64;
        int col_bits = 7;
        int bank_bits = 4;
        int ch_bits = 3;
        (void)row_bytes;
        uint64_t burst_idx = addr / burst_bytes;
        int bk = (burst_idx >> col_bits) & ((1 << bank_bits) - 1);
        int ch = (burst_idx >> (col_bits + bank_bits)) & ((1 << ch_bits) - 1);
        int row = burst_idx >> (col_bits + bank_bits + ch_bits);
        return {ch, bk, row};
    }

    std::vector<PIMInstruction> m_program;
    size_t m_pc = 0;
    uint64_t m_global_cycles = 0;
    std::unordered_map<int, uint64_t> m_per_bank_cycles;
    std::unordered_map<int, std::optional<int>> m_per_bank_open_row;

    int m_head_dim = 128;
    int m_ddr_row_bytes = 8192;
    int m_banks_per_pair = 1;
    int m_num_channels = 8;
    int m_num_banks = 16;
    int m_bytes_per_element = 2;
};

}   // namespace Ramulator
