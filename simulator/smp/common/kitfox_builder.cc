#include "kitfox_builder.h"

using namespace manifold::kitfox_proxy;

/*
KitFoxBuilder::KitFoxBuilder(const char* configFile, uint64_t samplingFreq) : m_config_file(configFile), m_clock_freq(samplingFreq)
{
    component_id = new int[1];
    dram_ranks = 0;
}
*/
KitFoxBuilder::KitFoxBuilder(const char* core_file, const char* hmc0, int num_pkgs, uint64_t samplingFreq, uint64_t num_ranks)
    : core_config_file(core_file), hmc0_config_file(hmc0), m_clock_freq(samplingFreq), num_files(num_pkgs), dram_ranks(num_ranks)
{
    component_id = new int [num_files];
    proxy = new kitfox_proxy_t* [num_files];
    for(int i = 0; i < num_files; i++)
    {
        proxy[i] = NULL;
        component_id[i] = 999 + i;
    }
    cerr << "core config file= " << core_config_file << " hmc0 config file= " << hmc0_config_file << endl;
}

KitFoxBuilder::KitFoxBuilder(const char* configFile, uint64_t samplingFreq) : core_config_file(configFile), m_clock_freq(samplingFreq)
{
    num_files = 1;
    dram_ranks = 0;
    proxy = new kitfox_proxy_t* [num_files];
    component_id = new int [num_files];
    proxy[0] = NULL;
    component_id[0] = 999;
    cerr << "core config file= " << core_config_file << endl;
}

void KitFoxBuilder::create_proxy(int hmc)
{
    if(hmc)
    {
        component_id[0] = manifold::kernel::Component::Create<kitfox_proxy_t>(0, core_config_file, m_clock_freq); // kitfox_proxy is in LP 0
        proxy[0] = manifold::kernel::Component :: GetComponent<kitfox_proxy_t>(component_id[0]);
        component_id[1] = manifold::kernel::Component::Create<kitfox_proxy_t>(0, hmc0_config_file, m_clock_freq, dram_ranks); // kitfox_proxy is in LP 0
        proxy[1] = manifold::kernel::Component :: GetComponent<kitfox_proxy_t>(component_id[1]);
    }
    else
    {
        component_id[0] = manifold::kernel::Component::Create<kitfox_proxy_t>(0, core_config_file, m_clock_freq); // kitfox_proxy is in LP 0
        proxy[0] = manifold::kernel::Component :: GetComponent<kitfox_proxy_t>(component_id[0]);
    }
}

KitFoxBuilder::~KitFoxBuilder()
{
    if (proxy)
        delete proxy;
}

/*
KitFoxBuilder::KitFoxBuilder(const char* configFile, uint64_t samplingFreq) : m_config_file(configFile), m_clock_freq(samplingFreq)
{
}

void KitFoxBuilder::create_proxy()
{
    component_id = manifold::kernel::Component::Create<kitfox_proxy_t>(0, m_config_file, m_clock_freq); // kitfox_proxy is in LP 0
    proxy = manifold::kernel::Component :: GetComponent<kitfox_proxy_t>(component_id);
}

KitFoxBuilder::~KitFoxBuilder()
{
    if (proxy)
        delete proxy;
}
*/
