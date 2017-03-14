#ifndef MANIFOLD_UARCH_DESTMAP_H
#define MANIFOLD_UARCH_DESTMAP_H

#include <stdint.h>
#include <assert.h>
#include <vector>
#include <iostream>

#ifdef HMCDEBUG
#include <ios>
#endif

namespace manifold {
namespace uarch {

class DestMap {
public:
    virtual ~DestMap() {}
    virtual int lookup(uint64_t) = 0;
    virtual uint64_t get_local_addr(uint64_t) = 0;
    virtual uint64_t get_global_addr(uint64_t, uint64_t) = 0;
    virtual int get_page_offset_bits (void) = 0;
};


class PageBasedMap : public manifold::uarch::DestMap {
public:
    PageBasedMap(std::vector<int>& nodes, int page_offset_bits) : m_page_offset_bits(page_offset_bits)
    {
        assert(nodes.size() > 0);
        for(unsigned i=0; i<nodes.size(); i++)
            m_nodes.push_back(nodes[i]);

        m_selector_bits = myLog2(nodes.size());

        m_selector_mask = (0x1 << m_selector_bits) - 1;
        m_page_offset_mask = (0x1 << m_page_offset_bits) - 1;
    }

    int lookup(uint64_t addr)
    {
        int idx = m_selector_mask & (addr >> m_page_offset_bits);
        int m_id = m_nodes[idx];

#ifdef HMCDEBUG
       	std::cerr << "VaultMap idx " << idx << " m_id " << m_id << std::endl;
#endif

        return m_id;
    }

    int get_idx(int m_id)
    {
        for (unsigned int i = 0; i < m_nodes.size(); i++) {
            if (m_nodes[i] == m_id)
	    {
#ifdef HMCDEBUG
			std::cerr << "VaultMap get_idx: " << i << std::endl;
#endif
                return i;
	    }
        }

        assert(0);
    }

    uint64_t get_local_addr(uint64_t addr)
    {
        uint64_t up_addr = addr >> (m_page_offset_bits + m_selector_bits);
        uint64_t low_addr = addr & m_page_offset_mask;
        uint64_t laddr = (up_addr << m_page_offset_bits) | low_addr;
#ifdef HMCDEBUG
		std::cerr << "VaultMap addr: " << std::hex << addr << " laddr " << laddr
                << std::dec << " m_page_offset_bits " << m_page_offset_bits << " m_selector_bits " << m_selector_bits
				<< " m_page_offset_mask " << std::hex << m_page_offset_mask << std::dec << std::endl;
#endif

        return laddr;
    }

    //fixme: assume the cache offset field is less than a page
    uint64_t get_global_addr(uint64_t addr, uint64_t m_id)
    {
        int idx = get_idx(m_id);

        uint64_t up_addr = addr >> m_page_offset_bits;
        uint64_t low_addr = (addr & m_page_offset_mask) | idx << (m_page_offset_bits);
        uint64_t gaddr = (up_addr << (m_page_offset_bits + m_selector_bits)) | low_addr;

#ifdef HMCDEBUG
		std::cerr << "VaultMap addr: " << std::hex << addr << " gaddr " << gaddr << " idx " << std::dec << idx << std::endl;
#endif

        return  gaddr;
    }

    int get_page_offset_bits()
    {
        return m_page_offset_bits;
    }

    int get_selector_bits()
    {
	return m_selector_bits;
    }

    uint64_t get_selector_mask()
    {
	return m_selector_mask;
    }

    int get_nodes_size()
    {
	return m_nodes.size();
    }

private:

    static int myLog2(unsigned num)
    {
        assert(num > 0);

        int bits = 0;
        while(((unsigned)0x1 << bits) < num) {
            bits++;
        }
        return bits;
    }

    std::vector<int> m_nodes; //target node ids
    const int m_page_offset_bits;
    int m_selector_bits;
    uint64_t m_selector_mask;
    uint64_t m_page_offset_mask;
};


class HMCDestMap : public manifold::uarch::DestMap {
public:
    HMCDestMap(std::vector<int>& hmc_nodes, int num_links, int max_HMC_units, int cache_line_size) :
    	num_serdes(num_links), MAX_HMC(max_HMC_units)
    {
        assert(hmc_nodes.size() > 0);

        for(unsigned i=0; i<hmc_nodes.size(); i++)
            m_nodes.push_back(hmc_nodes[i]);

        // log2(number of HMCs) = number of bits required to
        // select the HMC unit
        assert (hmc_nodes.size() % num_serdes == 0);

        int num_HMC = hmc_nodes.size() / num_serdes;
        assert(num_HMC <= MAX_HMC);

        m_selector_bits = myLog2(num_HMC);
        m_byte_offset_bits = myLog2(cache_line_size);

        m_selector_mask = (0x1 << m_selector_bits) - 1;
#ifdef HMCDEBUG
        std::cerr << std::dec << "HMCMap m_selector_bits: " << m_selector_bits << " m_byte_offset_bits: "
        		<< m_byte_offset_bits << std::hex << " m_selector_mask: " << m_selector_mask << std::dec << std::endl;
#endif
    }

    int lookup(uint64_t addr)
    {
    	uint64_t adj_addr = addr >> m_byte_offset_bits;
        int idx = m_selector_mask & (adj_addr);
        int m_id = 0;
        int p_rand = 0;

        /*
         * Assuming every HMC has same number of SerDes Links.
         * Once we know the HMC number to which the request
         * must be sent, we need to choose the dst_id i.e.
         * the network id.
         *
         * Eg: 	m_selector_mask = 0x7, addr = 0x 'something'
         * 		idx = 0x7 & addr;
         * Let idx = 5 i.e. the pkt must go to the 6th HMC
         * 		p_rand = some number in the range {0 - 4}
         * 				 Assuming each HMC has 4 links
         * say	p_rand = 3
         * 		m_id = m_nodes[3 + (4 * 5)]
         * 			 = m_nodes[23] which is the 4th network
         * 			 port of the 6th HMC
         */
        p_rand = rand() % num_serdes;
		assert( (p_rand + (num_serdes * idx)) <= m_nodes.size() );

#ifdef HMCDEBUG
        std::cerr << "HMCMap prand: " << p_rand << " idx: " << idx << " num serdes: " << num_serdes << " (p_rand + (num_serdes * idx)): " << (p_rand + (num_serdes * idx)) << std::endl;
#endif

        m_id = m_nodes[p_rand + (num_serdes * idx)];
        return m_id;
    }

    /*
     * This function is redundant, at least for HMC_map
     */

    int get_idx(int m_id)
    {
        for (int i = 0; i < m_nodes.size(); i++) {
            if (m_nodes[i] == m_id)
            {
#ifdef HMCDEBUG
            	std::cerr << "HMCMap get_idx: " << i << std::endl;
#endif
                return i;
            }
        }

        assert(0);
    }

    /*
     * Here we need to strip away the lower bits.
     * Suppose m_selector_bits = 2, i.e. there are 2, 3 or 4 HMCs
     * The local_addr that will be sent to 'that' HMC should not
     * contain the lower 2 bits. Extension is similar to 5 - 8 HMCs.
     */
    uint64_t get_local_addr(uint64_t addr)
    {
        uint64_t laddr = addr >> (m_selector_bits + m_byte_offset_bits);
#ifdef HMCDEBUG
		std::cerr << "HMCMap addr: " << std::hex << addr << " laddr " << laddr << std::dec << std::endl;
#endif
        return laddr;
    }

    /*
     * Here we need to append the addr at the LSB end by the HMC
     * index field.
     *
     * Suppose m_selector_bits = 2, i.e there are 2, 3 or 4 HMCs
     * The addr will not contain the HMC id. So depending on the
     * m_id i.e. the node_id of the pkt should be assigned.
     */
    uint64_t get_global_addr(uint64_t addr, uint64_t hmc_id)
    {
        uint64_t gaddr = (addr << m_selector_bits) | hmc_id;
        gaddr = gaddr << m_byte_offset_bits;
#ifdef HMCDEBUG
		std::cerr << "HMCMap addr: " << std::hex << addr << " gaddr " << gaddr << " hmc_id " << std::dec << hmc_id << std::endl;
#endif
        return  gaddr;
    }

    /*
     * This function is basically useless for HMCDestMap
     */
    int get_page_offset_bits()
    {
	return m_selector_bits;
    }

    int get_selector_bits()
    {
	return m_selector_bits;
    }

    int get_byte_offset_bits()
    {
	return m_byte_offset_bits;
    }

    uint64_t get_selector_mask()
    {
    	return m_selector_mask;
    }

    int get_nodes_size()
    {
    	return m_nodes.size();
    }

private:

    static int myLog2(unsigned num)
    {
        assert(num > 0);

        int bits = 0;
        while(((unsigned)0x1 << bits) < num) {
            bits++;
        }
        return bits;
    }

    std::vector<int> m_nodes; //target HMC node ids
    int num_serdes;
    int MAX_HMC;
    int m_selector_bits; // Number of bits required to select HMC instances
    uint64_t m_selector_mask; // Selector mask. Eg. 4 HMCs => 0x3, > 4 HMCs => 0x7
    int m_byte_offset_bits; // Number of bits required to address a cache line (16B, 32B, 64B = 4, 5, 6 bits resp)
};


} //namespace uarch
} //namespace manifold

#endif //MANIFOLD_UARCH_DESTMAP_H
