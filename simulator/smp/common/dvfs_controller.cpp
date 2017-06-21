#include "dvfs_controller.h"
#include "kernel/component.h"
#include "kernel/manifold.h"

using namespace manifold::kernel;
using namespace manifold::kitfox_proxy;
using namespace std;


namespace manifold {
namespace dvfs_controller {

Controller::Controller(int id, dvfs_settings& settings, manifold::kernel::Clock& clk)
    : controller_id(id), controller_clk(clk)
{
    p_spx = dynamic_cast<Spx_builder*>(settings.proc_builder);
    p_hmc = dynamic_cast<HMC_builder*>(settings.mem_builder);
//    p_kfb = settings.kitfox_builder;
    std::map<int,int>* proc_id_cid_map = p_spx->get_proc_id_cid_map();
    std::map<int,int>* serdes_id_cid_map = p_hmc->get_serdes_id_cid_map();


    for(map<int,int>::iterator it = proc_id_cid_map->begin(); it != proc_id_cid_map->end(); ++it)
    {
        proc_cid.push_back((*it).second);
        core.push_back(manifold::kernel::Component :: GetComponent<manifold::spx::spx_core_t>(proc_cid.back()));
    }


    for (int i = 0; i < p_hmc->get_num_hmcs(); i++)
    {
        for(map<int,int>::iterator it = serdes_id_cid_map[i].begin(); it != serdes_id_cid_map[i].end(); ++it)
        {
            serdes_cid.push_back((*it).second);
            serdes.push_back(manifold::kernel::Component :: GetComponent<manifold::hmc_serdes::HMC_SerDes>(serdes_cid.back()));
        }
    }

    //register with clock
    Clock :: Register(clk, this, &Controller::tick, (void(Controller::*)(void)) 0 );
}

void Controller::tick()
{
    Clock* clk;
    kitfox_proxy_t* p_kfp;
//    double dyn_average, leak_average;
    if (controller_clk.NowTicks() == 1000)
    {
//        p_kfp = p_kfb->get_kitfox(0);
        for(int i = 0; i < p_spx->get_num_procs(); i++)
        {
//            dyn_average = std::accumulate(p_kfp->cpu_dyn_pwr[i].begin(), p_kfp->cpu_dyn_pwr[i].end(), 0.0) / p_kfp->cpu_dyn_pwr[i].size();
//            leak_average = std::accumulate(p_kfp->cpu_leak_pwr[i].begin(), p_kfp->cpu_leak_pwr[i].end(), 0.0) / p_kfp->cpu_leak_pwr[i].size();
//            cerr << "Average over " << p_kfp->cpu_dyn_pwr[i].size() << "samples Core[" << i << "] dyn_pwr: " << dyn_average << " leakage: " << leak_average
//                 << " Total: " << dyn_average + leak_average << endl;
            clk = core[i]->get_clock();
            clk->set_frequency((double) 3000000000.0); // 3GHz
        }

//        p_kfp = p_kfb->get_kitfox(1);
        for(int i = 0; i < (p_hmc->get_num_serdes() * p_hmc->get_num_hmcs()); i++)
        {
//            dyn_average = std::accumulate(p_kfp->serdes_dyn_pwr[i].begin(), p_kfp->serdes_dyn_pwr[i].end(), 0.0) / p_kfp->serdes_dyn_pwr[i].size();
//            leak_average = std::accumulate(p_kfp->serdes_leak_pwr[i].begin(), p_kfp->serdes_leak_pwr[i].end(), 0.0) / p_kfp->serdes_leak_pwr[i].size();
//            cerr << "Average over " << p_kfp->serdes_dyn_pwr[i].size() << "samples SerDes[" << i << "] dyn_pwr: " << dyn_average << " leakage: " << leak_average
//                 << " Total: " << dyn_average + leak_average << endl;
            clk = serdes[i]->get_clock();
            clk->set_frequency((double) 15000000000.0); // 15GHz
        }

        print_stats(cerr);
    }
    else if (controller_clk.NowTicks() == 1500)
    {
        for(int i = 0; i < p_spx->get_num_procs(); i++)
        {
//            dyn_average = std::accumulate(p_kfp->cpu_dyn_pwr[i].begin(), p_kfp->cpu_dyn_pwr[i].end(), 0.0) / p_kfp->cpu_dyn_pwr[i].size();
//            leak_average = std::accumulate(p_kfp->cpu_leak_pwr[i].begin(), p_kfp->cpu_leak_pwr[i].end(), 0.0) / p_kfp->cpu_leak_pwr[i].size();
//            cerr << "Average over " << p_kfp->cpu_dyn_pwr[i].size() << "samples Core[" << i << "] dyn_pwr: " << dyn_average << " leakage: " << leak_average
//                 << " Total: " << dyn_average + leak_average << endl;
            clk = core[i]->get_clock();
            clk->set_frequency((double) 1000000000.0); // 3GHz
        }

        for(int i = 0; i < (p_hmc->get_num_serdes() * p_hmc->get_num_hmcs()); i++)
        {
//            dyn_average = std::accumulate(p_kfp->serdes_dyn_pwr[i].begin(), p_kfp->serdes_dyn_pwr[i].end(), 0.0) / p_kfp->serdes_dyn_pwr[i].size();
//            leak_average = std::accumulate(p_kfp->serdes_leak_pwr[i].begin(), p_kfp->serdes_leak_pwr[i].end(), 0.0) / p_kfp->serdes_leak_pwr[i].size();
//            cerr << "Average over " << p_kfp->serdes_dyn_pwr[i].size() << "samples SerDes[" << i << "] dyn_pwr: " << dyn_average << " leakage: " << leak_average
//                 << " Total: " << dyn_average + leak_average << endl;
            clk = serdes[i]->get_clock();
            clk->set_frequency((double) 10000000000.0); // 15GHz
        }

        print_stats(cerr);
    }
}

void Controller::compute_next_dvfs()
{
    // Go Crazy over here!
}

void Controller::print_stats(ostream& out)
{
    out << endl;
    for(int i = 0; i < p_spx->get_num_procs(); i++)
        out << "Core[" << i << "] freq= " << (core[i]->get_clock())->freq << endl;

    for(int i = 0; i < (p_hmc->get_num_serdes() * p_hmc->get_num_hmcs()); i++)
        out << "Serdes[" << i << "] freq= " << (serdes[i]->get_clock())->freq << endl;
    out << endl;
}

void Controller::print_config(ostream& out)
{
    out << " type: DVFS CONTROLLER" << endl;
    out << " Sampling Frequency: " << controller_clk.freq << " Hz" << endl;
}

} // namespace dvfs_controller
} // namespace manifold
