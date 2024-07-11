/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	Author: discord/bruh4096#4512

	This file contains declarations of member functions for flightplan interface class. 
    This class acts as a layer ontop of the flightplan class. Its job is to fetch data
    from appropriate navigation data bases and store it in the flightplan correctly.
*/


#pragma once

#include "flightplan.hpp"
#include <libnav/awy_db.hpp>
#include <libnav/str_utils.hpp>
#include <libnav/common.hpp>
#include <assert.h>

#include <iostream>

namespace test
{
    enum ProcType
    {
        PROC_TYPE_SID = 0,
        PROC_TYPE_STAR = 1,
        PROC_TYPE_APPCH = 2
    };

    constexpr size_t N_PROC_DB_SZ = 5;
    constexpr size_t N_LEG_SEG_BUF_SZ = 400;
    constexpr size_t N_ARR_DB_OFFSET = 2;
    constexpr size_t N_DFMS_ENRT_WORDS = 6;
    constexpr double DEFAULT_VS_FPM = 2000;
    constexpr double DEFAULT_GS_KTS = 250;
    const std::string NONE_TRANS = "NONE";
    const std::string MISSED_APPR_SEG_NM = "MISSED APPRCH";
    // X-Plane .fms format stuff
    constexpr char DFMS_COL_SEP = ' ';
    constexpr uint8_t N_DFMS_OUT_PREC = 6;
    const std::string DFMS_PADDING = "I\n1100 Version\n";
    const std::string DFMS_RWY_PREFIX = "RW";
    // .fms row headers:
    const std::string DFMS_AIRAC_CYCLE_NM = "CYCLE";
    const std::string DFMS_DEP_NM = "ADEP";
    const std::string DFMS_DEP_RWY_NM = "DEPRWY";
    const std::string DFMS_SID_NM = "SID";
    const std::string DFMS_SID_TRANS_NM = "SIDTRANS";
    const std::string DFMS_ARR_NM = "ADES";
    const std::string DFMS_ARR_RWY_NM = "DESRWY";
    const std::string DFMS_STAR_NM = "STAR";
    const std::string DFMS_STAR_TRANS_NM = "STARTRANS";
    const std::string DFMS_N_ENRT_NM = "NUMENR";

    const std::string DFMS_DIR_SEG_NM = "DRCT";

    const std::string DFMS_FILE_POSTFIX = ".fms";


    struct dfms_arr_data_t
    {
        std::string star, star_trans, arr_rwy, arr_icao;
    };

    struct leg_seg_t
    {
        bool is_arc, is_finite;
        geo::point start, end, arc_ctr;
        double turn_rad_nm;
    };


    std::string get_appr_rwy(std::string& appr);

    std::string get_dfms_rwy(std::string& rwy_nm);

    libnav::waypoint_t get_va_end_wpt(float va_alt_ft);

    geo::point compute_leg(geo::point start, double hdg_trk_diff_deg, leg_t *prev, 
        leg_t *curr, leg_t *next, leg_seg_t *out);


    class FplnInt: public FlightPlan
    {
    public:
        FplnInt(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::shared_ptr<libnav::AwyDB> aw_db, 
            std::string cifp_path);

        // Import from .fms file:

        libnav::DbErr load_from_fms(std::string& file_nm, bool set_arpts=true);

        // Export to .fms file:

        void save_to_fms(std::string& file_nm, bool save_sid_star=true);

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

        // Enroute:

        bool add_enrt_seg(timed_ptr_t<seg_list_node_t> next, std::string name);

        // End MUST be an airway id

        bool awy_insert(timed_ptr_t<seg_list_node_t> next, std::string end_id);

        bool delete_via(timed_ptr_t<seg_list_node_t> next);

        bool delete_seg_end(timed_ptr_t<seg_list_node_t> next);

        // Leg list interface functions:

        void dir_from_to(timed_ptr_t<leg_list_node_t> from, 
            timed_ptr_t<leg_list_node_t> to);

        void add_direct(libnav::waypoint_t wpt, timed_ptr_t<leg_list_node_t> next);

        bool delete_leg(timed_ptr_t<leg_list_node_t> next);

        ~FplnInt();

    private:
        std::string arr_rwy;

        std::vector<libnav::str_umap_t> proc_db;
        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::NavaidDB> navaid_db;

        libnav::arinc_rwy_db_t dep_rnw, arr_rnw;
        libnav::runway_entry_t dep_rnw_data, arr_rnw_data;

        double fpl_id_calc;

        leg_seg_t *leg_seg_buf;
        size_t n_leg_seg;


        // Static member functions:

        static size_t get_proc_db_idx(ProcType tp, bool is_arr=false);

        static fpl_segment_types get_proc_tp(ProcType tp);

        static fpl_segment_types get_trans_tp(ProcType tp);

        static std::vector<std::string> get_proc(libnav::str_umap_t& db, std::string rw="", 
            bool is_appch=false);

        static std::vector<std::string> get_proc_trans(std::string proc, libnav::str_umap_t& db, 
            libnav::arinc_rwy_db_t& rwy_db, bool is_rwy=false);

        static std::string get_dfms_enrt_leg(leg_list_node_t* lg, bool force_dir=false);

        // Non-static member functions:

        // Auxiliury functions for import from .fms:

        /*
            Function: process_dfms_term_line
            Description:
            Processes a single line of .fms file describing airports/procedures
            @param l_split: reference to the target split line
            @return error code
        */

        libnav::DbErr process_dfms_proc_line(std::vector<std::string>& l_split, 
            bool set_arpts, dfms_arr_data_t* arr_data);

        libnav::DbErr set_dfms_arr_data(dfms_arr_data_t* arr_data, bool set_arpt);

        bool get_dfms_wpt(std::vector<std::string>& l_split, libnav::waypoint_t* out);

        // Auxiliury functions for export to .fms:

        std::string get_dfms_arpt_leg(bool is_arr=false);
        
        size_t get_dfms_enrt_legs(std::vector<std::string>* out);

        // Other auxiliury functions:

        bool add_fpl_seg(libnav::arinc_leg_seq_t& legs, fpl_segment_types seg_tp, std::string seg_nm,
            seg_list_node_t *next=nullptr, bool set_ref=true);

        /*
            Function: get_awy_tf_leg
            Description:
            Makes a TF leg using a waypoint id taken from airway data base.
            @param wpt_id: id of the waypoint taken from airway data base. MUST be a valid id
            @return: arinc424 TF leg
        */

        leg_t get_awy_tf_leg(libnav::awy_point_t awy_pt);

        void add_awy_seg(std::string awy, seg_list_node_t *next, 
            std::vector<libnav::awy_point_t>& awy_pts);

        bool set_sid_star(std::string proc_nm, bool is_star=false, bool reset_rwy=true);

        bool set_appch_legs(std::string appch, std::string& arr_rwy, 
            libnav::arinc_leg_seq_t legs);

        bool set_appch(std::string appch);

        bool set_proc_trans(ProcType tp, std::string trans, bool is_arr=false);
    };
} // namespace test
