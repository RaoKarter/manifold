/*
 * sysBuilder_llp_HUT.h
 *
 *  Created on: Feb 14, 2017
 *      Author: Karthik Rao
 *
 *  This header file used for HMC unit test
 */

#ifndef SYSBUILDER_LLP_HUT_H_
#define SYSBUILDER_LLP_HUT_H_

#include <libconfig.h++>
#include <vector>
#include <set>
#include "kernel/manifold.h"
#include "kernel/component.h"
#include "uarch/DestMap.h"
#include "mc_builder.h"

// this data structure to hold a node's type and lp
enum {INVALID_NODE=0, EMPTY_NODE, CORE_NODE, MC_NODE, CORE_MC_NODE, L2_NODE};

struct Node_conf_llp {
	Node_conf_llp() : type(INVALID_NODE) {}

	int type;
	int lp;
};

class SysBuilder_llp_HUT {
public:
    
	SysBuilder_llp_HUT(const char* fname);
	~SysBuilder_llp_HUT();

	void config_system();
	void build_system(); 

	void print_config(std::ostream& out);
	void print_stats(std::ostream& out);

	libconfig::Config m_config;

	MemControllerBuilder* get_mc_builder() { return m_mc_builder; }

	manifold::kernel::Ticks_t get_stop_tick() { return STOP; }
	size_t get_mc_node_size() { return mc_node_idx_vec.size(); }

	manifold::kernel::Clock* get_default_clock() { return m_default_clock; }

protected:

	virtual void config_components();
	virtual void create_nodes();

	virtual void do_partitioning_1_part(int);

	MemControllerBuilder* m_mc_builder;
	int num_serdes;
	int MAX_VAULTS;
	int MAX_SERDES;
	manifold::kernel::Ticks_t STOP; //simulation stop time
	uint64_t m_DEFAULT_CLOCK_FREQ; //default clock's frequency
	int trans_size;

	std::vector<Node_conf_llp> m_node_conf;

	/*
	* For HMC, "mc_node_idx_vec" is the vector containing the node_ids
	* of all the HMCs connected to the network. Each HMC will have a fixed
	* number of SerDes links given by the length of xbar.node_idx configuration
	* input.
	*
	* "mc_node_idx_set" is the set which will contain the node_ids of all
	* the HMCs connected to the network.
	*
	* "mc_id_lp_map" contains the lp to node_id mapping for each HMC.
	* Currently, all the HMC nodes are on lp = 0;
	* !!!!!!!!!!!!!TODO: Need to figure out mpi implementation of HMC!!!!!!!!!!!!!
	*
	*/
	std::vector<int> mc_node_idx_vec;
	std::vector<int> vault_node_idx_vec;

	std::set<int> mc_node_idx_set; //set is used to ensure each index is unique
	std::set<int> vault_node_idx_set; //set is used to ensure each index is unique

	std::map<int, int> mc_id_lp_map; //maps mc's node id to its LP

private:

	void connect_components();
	void dep_injection_for_mcp();

	bool m_conf_read; //used to ensure config_system is called first

	manifold::kernel::Clock* m_default_clock;
};

#endif /* SYSBUILDER_LLP_HUT_H_ */
