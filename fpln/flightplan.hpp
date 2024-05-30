#pragma once

#include <libnav/cifp_parser.hpp>
#include <libnav/arpt_db.hpp>
#include <libnav/navaid_db.hpp>
#include <libnav/hold_db.hpp>
#include <libnav/awy_db.hpp>
#include <mutex>
#include <map>
#include "linked_list.hpp"


namespace test
{
    constexpr size_t N_FPL_LEG_CACHE_SZ = 200;
    constexpr size_t N_FPL_SEG_CACHE_SZ = 100;
    constexpr size_t N_FPL_REF_SZ = 9;

    enum fpl_segment_types
    {
        FPL_SEG_DEP_RWY = 1,
        FPL_SEG_SID = FPL_SEG_DEP_RWY + 1,
        FPL_SEG_SID_TRANS = FPL_SEG_SID + 1,
        FPL_SEG_ENRT = FPL_SEG_SID_TRANS + 1,
        FPL_SEG_STAR_TRANS = FPL_SEG_ENRT + 1,
        FPL_SEG_STAR = FPL_SEG_STAR_TRANS + 1,
        FPL_SEG_APPCH_TRANS = FPL_SEG_STAR + 1,
        FPL_SEG_APPCH = FPL_SEG_APPCH_TRANS + 1
    };

    struct leg_list_data_t;

    struct fpl_seg_t
    {
        bool direct;
        std::string name;
        fpl_segment_types seg_type;

        struct_util::list_node_t<leg_list_data_t> *end;
    };

    struct leg_list_data_t
    {
        int leg;
        bool is_discon;
        struct_util::list_node_t<fpl_seg_t> *seg;
    };

    struct fpl_ref_t
    {
        std::string name;
        struct_util::list_node_t<fpl_seg_t> *ptr;
    };

    
    static const struct struct_util::list_node_t<leg_list_data_t> EmptyNode = 
        {nullptr, nullptr, {}};
    static const struct struct_util::list_node_t<fpl_seg_t> EmptySeg = 
        {nullptr, nullptr, {}};
    static const struct fpl_ref_t EmptyRef = {"", nullptr};

    typedef struct_util::list_node_t<leg_list_data_t> leg_list_node_t;
    typedef struct_util::list_node_t<fpl_seg_t> seg_list_node_t;

    class FlightPlan
    {
        typedef std::unordered_map<std::string, std::set<std::string>> str_set_map_t;
        typedef std::map<int, leg_list_node_t*> leg_map_t;
        typedef std::map<std::string, seg_list_node_t*> seg_map_t;

    public:
        FlightPlan(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::string cifp_path);

        libnav::DbErr set_dep(std::string icao);

        std::string get_dep_icao();

        libnav::DbErr set_arr(std::string icao);

        std::string get_arr_icao();

        std::vector<std::string> get_dep_rwys();

        std::vector<std::string> get_arr_rwys();

        void print_seg();

        void print_legs();

        leg_map_t get_leg_map();

        seg_map_t get_seg_map();

        void delete_between(leg_list_node_t *start, leg_list_node_t *end);

        void delete_segment(seg_list_node_t *seg);

        void add_segment(std::vector<int>& legs, fpl_segment_types seg_tp,
            std::string seg_name, seg_list_node_t *next);

        //bool set_dep_rwy(std::string& rwy);

        //std::string get_dep_rwy();

        //str_set_map_t get_arr_appch();

        //bool set_arr_appch(std::string& proc_name, std::string& trans);

        ~FlightPlan();

    private:
        std::shared_ptr<libnav::ArptDB> arpt_db;
        std::shared_ptr<libnav::NavaidDB> navaid_db;
        std::string cifp_dir_path;

        libnav::Airport *departure, *arrival;

        std::vector<fpl_ref_t> fpl_refs;
        //fpl_ref_t dep_rwy;
        //std::string sid_name;
        //fpl_ref_t sid_trans;
        //fpl_ref_t enrt;
        //fpl_ref_t star;
        //fpl_ref_t star_trans;
        //fpl_ref_t appr;
        //fpl_ref_t appr_trans;


        struct_util::ll_node_stack_t<leg_list_node_t> leg_data_stack;
        struct_util::ll_node_stack_t<seg_list_node_t> seg_stack;

        struct_util::linked_list_t<leg_list_data_t> leg_list;
        struct_util::linked_list_t<fpl_seg_t> seg_list;
        std::mutex fpl_mtx;

        // WARNING: this does not lock flight plan mutex
        void reset_fpln();

        libnav::DbErr set_arpt(std::string icao, libnav::Airport **ptr);

        

        void add_legs(std::vector<int>& legs, fpl_segment_types seg_tp,
            std::string seg_name);
    };
};
