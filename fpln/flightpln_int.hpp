#pragma once

#include "flightplan.hpp"
#include <assert.h>


namespace test
{
    enum ProcType
    {
        PROC_TYPE_SID = 0,
        PROC_TYPE_STAR = 1,
        PROC_TYPE_APPCH = 2
    }

    constexpr size_t N_PROC_DB_SZ = 5;
    constexpr size_t N_ARR_DB_OFFSET = 2;


    class FplnInt: public FlightPlan
    {
    public:
        FplnInt(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::string cifp_path);

        // Airport functions:

        libnav::DbErr set_dep(std::string icao);

        std::string get_dep_icao();

        libnav::DbErr set_arr(std::string icao);

        std::string get_arr_icao();

        // Runway functions:

        std::vector<std::string> get_dep_rwys();

        std::vector<std::string> get_arr_rwys();

        bool set_dep_rwy(std::string& rwy);

        std::string get_dep_rwy();

        bool set_arr_rwy(std::string& rwy);

        std::string get_arr_rwy();

        // Airport procedure functions:

        std::vector<std::string> get_arpt_proc(ProcType tp, bool is_arr=false);

        std::vector<std::string> get_arpt_proc_trans(ProcType tp, bool is_rwy=false, bool is_arr=false);

    private:
        std::string arr_rwy;

        std::vector<libnav::str_umap_t> proc_db;

        libnav::arinc_rwy_db_t dep_rnw, arr_rnw;


        static size_t get_proc_db_idx(ProcType tp, bool is_arr=false);

        static std::vector<std::string> get_proc(std::string rw="", libnav::str_umap_t& db);

        static std::vector<std::string> get_proc_trans(std::string proc, libnav::str_umap_t& db, 
            libnav::arinc_rwy_db_t& rwy_db, bool is_rwy=false);
    };
} // namespace test
