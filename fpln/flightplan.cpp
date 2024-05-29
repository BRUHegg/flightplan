#include "flightplan.hpp"
#include "linked_list.hpp"


namespace test
{
    // FlightPlan class definitions:
    // Public member functions:

    FlightPlan::FlightPlan(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::string cifp_path): 
        leg_data_stack(N_FPL_LEG_CACHE_SZ), seg_stack(N_FPL_SEG_CACHE_SZ), 
        leg_list(), seg_list()
    {
        departure = nullptr;
        arrival = nullptr;

        arpt_db = apt_db;
        navaid_db = nav_db;

        cifp_dir_path = cifp_path;
    }

    libnav::DbErr FlightPlan::set_dep(std::string icao)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        libnav::DbErr out = set_arpt(icao, &departure);
        if(arrival != nullptr && departure != nullptr && departure->icao_code == icao
            && out != libnav::DbErr::ERR_NONE)
        {
            delete arrival;
            arrival = nullptr;
        }
        return out;
    }

    std::string FlightPlan::get_dep_icao()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure != nullptr)
            return departure->icao_code;
        return "";
    }

    libnav::DbErr FlightPlan::set_arr(std::string icao)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure == nullptr)
        {
            return libnav::DbErr::ERR_NONE;
        }
        
        return set_arpt(icao, &arrival);
    }

    std::string FlightPlan::get_arr_icao()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(arrival != nullptr)
            return arrival->icao_code;
        return "";
    }

    FlightPlan::~FlightPlan()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        reset_fpln();
        leg_data_stack.destroy();
        seg_stack.destroy();
    }

    // Private member functions:

    void FlightPlan::reset_fpln()
    {
        leg_list.release_all(leg_data_stack.ptr_stack);
        seg_list.release_all(seg_stack.ptr_stack);
    }

    libnav::DbErr FlightPlan::set_arpt(std::string icao, libnav::Airport **ptr)
    {
        if(*ptr != nullptr && (*ptr)->icao_code == icao)
        {
            return libnav::DbErr::ERR_NONE;
        }
        libnav::Airport *tmp = new libnav::Airport(icao, arpt_db, 
            navaid_db, cifp_dir_path);
        libnav::DbErr err_cd = tmp->err_code;
        if(err_cd != libnav::DbErr::SUCCESS && err_cd != libnav::DbErr::PARTIAL_LOAD)
        {
            delete tmp;
        }
        else
        {
            if(*ptr != nullptr)
            {
                reset_fpln();
                delete *ptr;
            }

            *ptr = tmp;
        }
        
        return err_cd;
    }
} // namespace test
