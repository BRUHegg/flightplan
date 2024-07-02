#pragma once

#include "flightplan.hpp"
#include <libnav/awy_db.hpp>
#include <assert.h>


namespace test
{
    enum ProcType
    {
        PROC_TYPE_SID = 0,
        PROC_TYPE_STAR = 1,
        PROC_TYPE_APPCH = 2
    };

    constexpr size_t N_PROC_DB_SZ = 5;
    constexpr size_t N_ARR_DB_OFFSET = 2;
    const std::string NONE_TRANS = "NONE";


    std::string get_appr_rwy(std::string& appr);


    class FplnInt: public FlightPlan
    {
    public:
        FplnInt(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::shared_ptr<libnav::AwyDB> aw_db, 
            std::string cifp_path);

        // Airport functions:

        libnav::DbErr set_dep(std::string icao);

        std::string get_dep_icao();

        libnav::DbErr set_arr(std::string icao);

        std::string get_arr_icao();

        // Runway functions:

        std::vector<std::string> get_dep_rwys(bool filter_rwy=false, bool filter_sid=false);

        std::vector<std::string> get_arr_rwys();

        bool set_dep_rwy(std::string& rwy);

        std::string get_dep_rwy();

        bool set_arr_rwy(std::string& rwy);

        std::string get_arr_rwy();

        // Airport procedure functions:

        std::vector<std::string> get_arpt_proc(ProcType tp, bool is_arr=false, 
            bool filter_rwy=false, bool filter_proc=false);

        std::vector<std::string> get_arpt_proc_trans(ProcType tp, bool is_rwy=false, bool is_arr=false);

        bool set_arpt_proc(ProcType tp, std::string proc_nm, bool is_arr=false);

        bool set_arpt_proc_trans(ProcType tp, std::string trans, bool is_arr=false);

        bool add_enrt_seg(timed_ptr_t<seg_list_node_t> next, std::string name);

        // End MUST be an airway id

        bool awy_insert(timed_ptr_t<seg_list_node_t> next, std::string end_id);

    private:
        std::string arr_rwy;

        std::vector<libnav::str_umap_t> proc_db;
        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::NavaidDB> navaid_db;

        libnav::arinc_rwy_db_t dep_rnw, arr_rnw;


        static size_t get_proc_db_idx(ProcType tp, bool is_arr=false);

        static fpl_segment_types get_proc_tp(ProcType tp);

        static fpl_segment_types get_trans_tp(ProcType tp);

        static std::vector<std::string> get_proc(libnav::str_umap_t& db, std::string rw="", 
            bool is_appch=false);

        static std::vector<std::string> get_proc_trans(std::string proc, libnav::str_umap_t& db, 
            libnav::arinc_rwy_db_t& rwy_db, bool is_rwy=false);

        bool add_fpl_seg(libnav::arinc_leg_seq_t& legs, fpl_segment_types seg_tp, std::string ref_nm,
            seg_list_node_t *next=nullptr);

        bool set_sid_star(std::string proc_nm, bool is_star=false);

        bool set_appch(std::string appch);

        bool set_proc_trans(ProcType tp, std::string trans, bool is_arr=false);
    };
} // namespace test
