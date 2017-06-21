#ifndef KITFOX_PROXY_BUILDER_H
#define KITFOX_PROXY_BUILDER_H

#define USE_KITFOX 1

#include "kernel/component-decl.h"
#include "kitfox_proxy.h"

class KitFoxBuilder {

public:
/*
    KitFoxBuilder(const char* configFile, uint64_t samplingFreq);
        : m_config_file(configFile), m_clock_freq(samplingFreq)
    {
        component_id = new int[1];
        component_id[0] = 999;
        dram_ranks = 0;
    }

*/
    KitFoxBuilder(const char* core_file, const char* hmc0, int num_pkgs, uint64_t samplingFreq, uint64_t num_ranks);
    KitFoxBuilder(const char* configFile, uint64_t s);
    virtual ~KitFoxBuilder();

    int get_component_id(int i) const { return component_id[i]; }
//    void create_proxy();
    void create_proxy(int hmc);
//    manifold::kitfox_proxy::kitfox_proxy_t* get_kitfox() const { return proxy; }
     manifold::kitfox_proxy::kitfox_proxy_t* get_kitfox(int i) const { return proxy[i]; }

protected:
    manifold::kitfox_proxy::kitfox_proxy_t **proxy;
    const char* core_config_file;
    const char* hmc0_config_file;
//    const char* m_config_file;
//    const char* m_config_file_hmc;
    const uint64_t m_clock_freq;
    int* component_id;
    int dram_ranks;

    int num_files;
};

/*
class SysBuilder_llp;

class KitFoxBuilder {
public:
    KitFoxBuilder(const char* configFile, uint64_t s);
    virtual ~KitFoxBuilder();

    int get_component_id() const { return component_id; }
    void create_proxy();
    manifold::kitfox_proxy::kitfox_proxy_t* get_kitfox() const { return proxy; }

protected:
    manifold::kitfox_proxy::kitfox_proxy_t *proxy;
    const char* m_config_file;
    const uint64_t m_clock_freq;

    int component_id;
};
*/

#endif //KITFOX_PROXY_BUILDER_H
