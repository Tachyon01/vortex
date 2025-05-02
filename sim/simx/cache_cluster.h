// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "cache_sim.h"

namespace vortex {

class CacheCluster : public SimObject<CacheCluster> {
public:
	std::vector<std::vector<SimPort<MemReq>>> CoreReqPorts;
	std::vector<std::vector<SimPort<MemRsp>>> CoreRspPorts;
	std::vector<SimPort<MemReq>> MemReqPorts;
	std::vector<SimPort<MemRsp>> MemRspPorts;
	#ifdef VM_ENABLE
	HashTable<MemReq*> pending_tlb_;
	#endif

	CacheCluster(const SimContext& ctx,
							const char* name,
							uint32_t num_inputs,
							uint32_t num_units,
							const CacheSim::Config& cache_config)
		: SimObject(ctx, name)
		, CoreReqPorts(num_inputs, std::vector<SimPort<MemReq>>(cache_config.num_inputs, this))
		, CoreRspPorts(num_inputs, std::vector<SimPort<MemRsp>>(cache_config.num_inputs, this))
		, MemReqPorts(cache_config.mem_ports, this)
		, MemRspPorts(cache_config.mem_ports, this)
		, caches_(MAX(num_units, 0x1)) 
		#ifdef VM_ENABLE
		, pending_tlb_(NUM_TLB_REQ)
        , tlb_(MAX(num_units, 0x1))       // Initialize tlb_ similarly when VM_ENABLE is defined
    	#endif
	{

		CacheSim::Config cache_config2(cache_config);
		if (0 == num_units) {
			num_units = 1;
			cache_config2.bypass = true;
		}

		char sname[100];

		// Arbitrate incoming core interfaces
		std::vector<MemArbiter::Ptr> input_arbs(cache_config.num_inputs);
		for (uint32_t i = 0; i < cache_config.num_inputs; ++i) {
			snprintf(sname, 100, "%s-input-arb%d", name, i);
			input_arbs.at(i) = MemArbiter::Create(sname, ArbiterType::RoundRobin, num_inputs, num_units);
			for (uint32_t j = 0; j < num_inputs; ++j) {
				this->CoreReqPorts.at(j).at(i).bind(&input_arbs.at(i)->ReqIn.at(j));
				input_arbs.at(i)->RspIn.at(j).bind(&this->CoreRspPorts.at(j).at(i));
			}
		}

		// Arbitrate outgoing memory interfaces
		std::vector<MemArbiter::Ptr> mem_arbs(cache_config.mem_ports);
		for (uint32_t i = 0; i < cache_config.mem_ports; ++i) {
			snprintf(sname, 100, "%s-mem-arb%d", name, i);
			#ifdef VM_ENABLE
			mem_arbs.at(i) = MemArbiter::Create(sname, ArbiterType::RoundRobin, 2*num_units, 1);
			#else
			mem_arbs.at(i) = MemArbiter::Create(sname, ArbiterType::RoundRobin, num_units, 1);
			#endif
			mem_arbs.at(i)->ReqOut.at(0).bind(&this->MemReqPorts.at(i));
			this->MemRspPorts.at(i).bind(&mem_arbs.at(i)->RspOut.at(0));
		}

		// Start looping for each cache
		for (uint32_t i = 0; i < num_units; ++i) {
			// Create caches for the unit
			snprintf(sname, 100, "%s-cache%d", name, i);
			caches_.at(i) = CacheSim::Create(sname, cache_config2);

			// If VM_ENABLE is defined, create and connect TLB
            #ifdef VM_ENABLE
            snprintf(sname, 100, "%s-tlb%d", name, i);
            tlb_.at(i) = TlbSim::Create(sname, 1, 1, CacheSim::Config{
                !VM_ENABLED,
                log2ceil(TLB_SIZE),    // C
                log2ceil(TLB_LINE_SIZE), // L
                log2ceil(TLB_WORD_SIZE), // W
                log2ceil(TLB_NUM_WAYS), // A num ways
                log2ceil(TLB_NUM_BANKS),// B
                XLEN,                   // address bits
                1,                      // number of ports
                cache_config.num_inputs,// number of inputs
                cache_config.mem_ports, // memory ports
                false,                  // write-back
                false,                  // write response
                TLB_MSHR_SIZE,          // mshr size
                2,                      // pipeline latency
            });

			// Connect the TLB to each cache
            // ReqOut from input_arbiter connected to CoreReqPort of TLB
            for (uint32_t j = 0; j < cache_config.num_inputs; ++j) {
				
				// receive tlb response
				auto& tlb_rsp_port = tlb_.at(i)->CoreRspPorts.at(j);
				if (!tlb_rsp_port.empty()) {
					auto& mem_rsp = tlb_rsp_port.front();
					// Look up the original request from the pending_tlb_ hash table using the response tag
					auto req = pending_tlb_.at(mem_rsp.tag);

					MemReq cache_req = *req;
					cache_req.addr = req->p_addr;       // Physical address from the original request
					// Later on instead of using this from the request use the addr from the response
					// And include the translation in the tlb module
					// cache_req.p_addr = mem_rsp.addr;       // Physical address from the TLB response

					// Forward the new MemReq to the cache
					caches_.at(i)->CoreReqPorts.at(j).push(cache_req, 2);
					DT(3, "tlb-rsp: addr=0x" << std::hex << mem_rsp.addr
						<< ", tag=0x" << mem_rsp.tag << std::dec << ", " << *req);
					pending_tlb_.release(mem_rsp.tag);
					tlb_rsp_port.pop();
					// Add performance counter here 
					// --pending_tlb_req;
				}

				// send tlb request
				auto& arb_req_port = input_arbs.at(j)->ReqOut.at(i);
				if (!arb_req_port.empty()) {
					const auto& mem_req = arb_req_port.front();

					// Clone the request and assign a new tag just for tracking
					MemReq tlb_req = mem_req;
					tlb_req.tag = pending_tlb_.allocate(mem_req);  // store the original request

					tlb_.at(i)->CoreReqPorts.at(j).push(tlb_req, 2); // Send the request to TLB 
					DT(3, "tlb-req: unit=" << i << ", port=" << j
						<< ", addr=0x" << std::hex << tlb_req.addr
						<< ", tag=0x" << tlb_req.tag << std::dec);
					arb_req_port.pop();
					// Add performance counter here 
					// ++pending_tlb_req;
					// ++pending_tlb_req;
				}

                caches_.at(i)->CoreRspPorts.at(j).bind(&input_arbs.at(j)->RspOut.at(i));
            }

			// Connect caches and tlb to the memory arbiters
			for (uint32_t j = 0; j < cache_config.mem_ports; ++j) {
				caches_.at(i)->MemReqPorts.at(j).bind(&mem_arbs.at(j)->ReqIn.at(2*i));
				tlb_.at(i)->MemReqPorts.at(j).bind(&mem_arbs.at(j)->ReqIn.at(2*i+1));
				mem_arbs.at(j)->RspIn.at(2*i).bind(&caches_.at(i)->MemRspPorts.at(j));
				mem_arbs.at(j)->RspIn.at(2*i+1).bind(&tlb_.at(i)->MemRspPorts.at(j));
			}

            #else
            // Directly connect input arbiters to caches when TLB is not enabled
			for (uint32_t j = 0; j < cache_config.num_inputs; ++j) {
				input_arbs.at(j)->ReqOut.at(i).bind(&caches_.at(i)->CoreReqPorts.at(j));
				caches_.at(i)->CoreRspPorts.at(j).bind(&input_arbs.at(j)->RspOut.at(i));
			}

			// Connect caches to the memory arbiters
			for (uint32_t j = 0; j < cache_config.mem_ports; ++j) {
				caches_.at(i)->MemReqPorts.at(j).bind(&mem_arbs.at(j)->ReqIn.at(i));
				mem_arbs.at(j)->RspIn.at(i).bind(&caches_.at(i)->MemRspPorts.at(j));
			}
			#endif
		}
	}

	~CacheCluster() {}

	void reset() {}

	void tick() {}

	CacheSim::PerfStats perf_stats() const {
		CacheSim::PerfStats perf;
		for (auto cache : caches_) {
			perf += cache->perf_stats();
		}
		return perf;
	}

private:
  std::vector<CacheSim::Ptr> caches_;
  #ifdef VM_ENABLE
    std::vector<TlbSim::Ptr> tlb_;  // TLBs are only defined if VM_ENABLE is defined
  #endif
};

}
