#include <iostream>
#include <memory>
#include <string>
#include "fpln/flightpln_int.hpp"
#include <libnav/geo_utils.hpp>

#define UNUSED(x) (void)(x)


double AC_LAT_DEF = 45.588670483;
double AC_LON_DEF = -122.598150383;


namespace test
{
    class Avionics
    {
    public:
        double ac_lat;
        double ac_lon;
        int legs_sbpg;

        double leg_list_id;
        double seg_list_id;

        bool flt_rwy, flt_proc, flt_trans;

        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::HoldDB> hold_db;

        std::shared_ptr<FplnInt> fpl;

        std::pair<size_t, double> leg_sel_cdu_l;
        std::pair<size_t, double> leg_sel_cdu_r;

        std::unordered_map<std::string, std::string> env_vars;

        std::string cifp_dir_path;


        Avionics(std::string apt_dat, std::string custom_apt, std::string custom_rnw,
            std::string fix_data, std::string navaid_data, std::string awy_data,
            std::string hold_data, 
            std::string cifp_path, double def_lat=AC_LAT_DEF, 
            double def_lon=AC_LON_DEF)
        {
            env_vars["ac_lat"] = strutils::double_to_str(def_lat, 8);
            env_vars["ac_lon"] = strutils::double_to_str(def_lon, 8);
            env_vars["legs_sbpg"] = "0";

            leg_list_id = -1;
            seg_list_id = -1;

            flt_rwy = false;
            flt_proc = false;
            flt_trans = false;

            cifp_dir_path = cifp_path;

            ac_lat = def_lat;
            ac_lon = def_lon;
            legs_sbpg = 0;

            arpt_db_ptr = 
                std::make_shared<libnav::ArptDB>(apt_dat, custom_apt, custom_rnw);
	        navaid_db_ptr = 
                std::make_shared<libnav::NavaidDB>(fix_data, navaid_data);
            awy_db = std::make_shared<libnav::AwyDB>(awy_data);
            hold_db = std::make_shared<libnav::HoldDB>(hold_data);

            libnav::DbErr err_arpt = arpt_db_ptr->get_err();
            libnav::DbErr err_wpt = navaid_db_ptr->get_wpt_err();
            libnav::DbErr err_nav = navaid_db_ptr->get_navaid_err();
            libnav::DbErr err_awy = awy_db->get_err();
            libnav::DbErr err_hold = hold_db->get_err();

            std::cout << navaid_db_ptr->get_wpt_cycle() << " " <<
                navaid_db_ptr->get_navaid_cycle() << " " << 
                awy_db->get_airac() << " " << hold_db->get_airac() << "\n";

            std::cout << "Fix data base version: " << 
                navaid_db_ptr->get_wpt_version() << "\n";
            std::cout << "Navaid data base version: " << 
                navaid_db_ptr->get_navaid_version() << "\n";

            if(err_arpt != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load airport database\n";
            }
            if(err_wpt != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load waypoint database\n";
            }
            if(err_nav != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load navaid database\n";
            }
            if(err_awy != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load airway database\n";
            }
            if(err_hold != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load hold database\n";
            }

            fpl = std::make_shared<FplnInt>(arpt_db_ptr, navaid_db_ptr, awy_db, cifp_dir_path);

            leg_sel_cdu_l = {0, 0};
            leg_sel_cdu_r = {0, 0};

            n_act_seg_list_sz = 0;
            n_act_leg_list_sz = 0;

            cap_ctr_idx = 0;
            fo_ctr_idx = 0;

            fpl_id_last = 0;
        }

        std::vector<list_node_ref_t<fpl_seg_t>> get_seg_list(size_t *sz)
        {
            *sz = n_act_seg_list_sz;
            return seg_list;
        }

        std::vector<list_node_ref_t<leg_list_data_t>> get_leg_list(size_t *sz)
        {
            *sz = n_act_leg_list_sz;
            return leg_list;
        }

        size_t get_nd_seg(nd_leg_data_t *out, size_t n_max)
        {
            size_t n_written = 0;
            for(size_t i = 0; i < n_act_leg_list_sz; i++)
            {
                if(!n_max)
                    return n_written;
                
                nd_leg_data_t tmp;
                tmp.leg_data = leg_list[i].data.misc_data;
                tmp.arc_ctr = leg_list[i].data.leg.center_fix.data.pos;
                tmp.end_name = leg_list[i].data.leg.main_fix.id;
                out[i] = tmp;

                n_max--;
            }
        }

        bool get_ctr(geo::point *out, bool fo_side)
        {
            size_t curr_idx = cap_ctr_idx;

            if(fo_side)
                curr_idx = fo_ctr_idx;

            if(curr_idx < n_act_leg_list_sz)
            {
                bool has_pos = leg_list[curr_idx].data.leg.has_main_fix;
                if(has_pos)
                {
                    *out = leg_list[curr_idx].data.leg.main_fix.data.pos;
                    return true;
                }
            }

            return false;
        }

        void step_ctr(bool bwd, bool fo_side)
        {
            if(!n_act_leg_list_sz)
                return;
            
            size_t *curr_idx = &cap_ctr_idx;

            if(fo_side)
                curr_idx = &fo_ctr_idx;

            if(bwd)
            {
                if(*curr_idx)
                    *curr_idx--;
                else
                    *curr_idx = n_act_leg_list_sz;
            }
            else
            {
                if(*curr_idx < n_act_leg_list_sz)
                    *curr_idx++;
                else
                    *curr_idx = 0;
            }
        }

        void update()
        {
            update_pos();
            fpl->update(0);

            update_lists();
        }

        ~Avionics()
        {
            hold_db.reset();
            awy_db.reset();
            navaid_db_ptr.reset();
            navaid_db_ptr.reset();
            arpt_db_ptr.reset();
        }

    private:
        std::vector<list_node_ref_t<fpl_seg_t>> seg_list;
        size_t n_act_seg_list_sz;
        std::vector<list_node_ref_t<leg_list_data_t>> leg_list;
        size_t n_act_leg_list_sz;

        size_t cap_ctr_idx, fo_ctr_idx;
        double fpl_id_last;


        void update_seg_list()
        {
            n_act_seg_list_sz = fpl->get_seg_list_sz();
            seg_list_id = fpl->get_sl_seg(0, n_act_seg_list_sz, &seg_list);
        }

        void update_leg_list()
        {
            n_act_leg_list_sz = fpl->get_leg_list_sz();
            leg_list_id = fpl->get_ll_seg(0, n_act_leg_list_sz, &leg_list);

            if(cap_ctr_idx >= n_act_leg_list_sz && n_act_leg_list_sz != 0)
            {
                cap_ctr_idx = n_act_leg_list_sz-1;
            }

            if(fo_ctr_idx >= n_act_leg_list_sz && n_act_leg_list_sz != 0)
            {
                fo_ctr_idx = n_act_leg_list_sz-1;
            }
        }

        void update_lists()
        {
            double fpl_id_curr = fpl->get_id();
            if(fpl_id_curr != fpl_id_last)
            {
                update_seg_list();
                update_leg_list();
            }
                    
            fpl_id_last = fpl_id_curr;
        }

        void update_pos()
        {
            bool lat_valid = strutils::is_numeric(env_vars["ac_lat"]);
            bool lon_valid = strutils::is_numeric(env_vars["ac_lon"]);
            bool sbp_vaild = strutils::is_numeric(env_vars["legs_sbpg"]);
            
            if(lon_valid && lat_valid)
            {
                ac_lat = std::stod(env_vars["ac_lat"]);
                ac_lon = std::stod(env_vars["ac_lon"]);
            }
            if(sbp_vaild)
            {
                legs_sbpg = std::stoi(env_vars["legs_sbpg"]);
            }
        }
    };

    struct awy_filter_data_t
    {
        std::string s;
        Avionics* ptr;
    };

    typedef void (*cmd)(Avionics*, std::vector<std::string>&);


    inline libnav::waypoint_entry_t select_desired(std::string& name,
            std::vector<libnav::waypoint_entry_t>& wpts)
    {
        if(wpts.size() == 0)
        {
            return {};
        }
        if(wpts.size() == 1)
        {
            return wpts[0];
        }
        std::cout << "Select desired " << name << "\n";
        for(size_t i = 0; i < wpts.size(); i++)
        {
            std::cout << i+1 << ". " << strutils::lat_to_str(wpts[i].pos.lat_rad 
                * geo::RAD_TO_DEG) 
                << " " << strutils::lat_to_str(wpts[i].pos.lon_rad
                * geo::RAD_TO_DEG) << "\n";
        }
        while(1)
        {
            std::string tmp;
            std::getline(std::cin, tmp);

            size_t num = size_t(strutils::stoi_with_strip(tmp));
            if(num != 0 && num < wpts.size() + 1)
            {
                return wpts[num-1];
            }
        }
    }

    inline void set_var(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Command expects 2 arguments: <variable name>, <value>\n";
            return;
        }

        av->env_vars[in[0]] = in[1];
    }

    inline void print(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: <variable name>\n";
            return;
        }

        if(av->env_vars.find(in[0]) != av->env_vars.end())
        {
            std::cout << av->env_vars[in[0]] << "\n";
        }
        else
        {
            std::cout << "Variable not found\n";
        }
    }

    inline void quit(Avionics* av, std::vector<std::string>& in)
    {
        UNUSED(av);

        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        std::exit(0);
    }

    inline void load_fpln(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        std::string dep_nm = av->fpl->get_dep_icao();
        std::string arr_nm = av->fpl->get_arr_icao();

        if(dep_nm != "" && arr_nm != "")
        {
            std::string file_nm = "flightplans/"+dep_nm+arr_nm;
            libnav::DbErr err = av->fpl->load_from_fms(file_nm, false);

            if(err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD)
            {
                std::cout << "Failed to load flight plan\n";
            }
        }
    }

    inline void save_fpln(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        std::string dep_nm = av->fpl->get_dep_icao();
        std::string arr_nm = av->fpl->get_arr_icao();

        if(dep_nm != "" && arr_nm != "")
        {
            std::string out_nm = "flightplans/"+dep_nm+arr_nm;
            av->fpl->save_to_fms(out_nm);
        }
    }

    inline void set_filter(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: {filter type(0 - runway, 1 - procedure, 2 - transition)}\n";
            return;
        }

        int flt_type = strutils::stoi_with_strip(in[0]);
        if(flt_type == 0)
        {
            av->flt_rwy = !(av->flt_rwy);
        }
        else if(flt_type == 1)
        {
            av->flt_proc = !(av->flt_proc);
        }
        else if(flt_type == 2)
        {
            av->flt_trans = !(av->flt_trans);
        }
        else
        {
            std::cout << "Filter type out of range\n";
        }
    }

    inline void fplinfo(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        std::cout << "Departure: " << av->fpl->get_dep_icao() << "\n";
        std::cout << "Arrival: " << av->fpl->get_arr_icao() << "\n";
        std::cout << "Departure runway: " << av->fpl->get_dep_rwy() << "\n";
        std::cout << "Arrival runway: " << av->fpl->get_arr_rwy() << "\n";
    }

    inline void set_fpl_dep(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: icao code\n";
            return;
        }

        libnav::DbErr err = av->fpl->set_dep(in[0]);
        if(err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid entry\n";
        }
        else if(err == libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Airport partially loaded\n";
        }
    }

    inline void set_fpl_arr(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: icao code\n";
            return;
        }

        libnav::DbErr err = av->fpl->set_arr(in[0]);
        if(err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid entry\n";
        }
        else if(err == libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Airport partially loaded\n";
        }
    }

    inline void set_dep_rwy(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: icao code\n";
            return;
        }

        bool rwy_set = av->fpl->set_dep_rwy(in[0]);

        if(!rwy_set)
        {
            std::cout << "Runway not set";
        }
    }

    inline void set_arr_rwy(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: icao code\n";
            return;
        }

        bool rwy_set = av->fpl->set_arr_rwy(in[0]);

        if(!rwy_set)
        {
            std::cout << "Runway not set";
        }
    }

    inline void get_dep_rwys(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 0)
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        std::vector<std::string> rwys = av->fpl->get_dep_rwys(av->flt_rwy, av->flt_proc);
        for(auto i: rwys)
        {
            std::cout << i << "\n";
        }
    }

    inline void get_arr_rwys(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 0)
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        std::vector<std::string> rwys = av->fpl->get_arr_rwys();
        for(auto i: rwys)
        {
            std::cout << i << "\n";
        }
    }

    inline void get_proc(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 3)
        {
            std::cout << "Command expects 3 arguments: {procedure type}, {DEP/ARR}, \
            {PROC/TRANS}\n";
            return;
        }

        int tmp = strutils::stoi_with_strip(in[0]);

        if(tmp < 0 || tmp > 2)
        {
            std::cout << "procedure type entry out of range\n";
            return;
        }

        bool is_arr = in[1] != "DEP";
        bool is_trans = in[2] == "TRANS";

        if((in[1] == "ARR" || in[1] == "DEP") && (in[2] == "TRANS" || in[2] == "PROC"))
        {
            std::vector<std::string> procs;
            if(!is_trans)
            {
                procs = av->fpl->get_arpt_proc(ProcType(tmp), is_arr, 
                    av->flt_rwy, av->flt_proc);
            }
            else
            {
                procs = av->fpl->get_arpt_proc_trans(ProcType(tmp), false, is_arr);
            }
            
            for(auto i: procs)
            {
                std::cout << i << "\n";
            }
        }
    }

    inline void set_proc(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 4)
        {
            std::cout << "Command expects 4 arguments: {procedure type}, {proc name}, \
                {DEP/ARR}, {TRANS/PROC}\n";
            return;
        }

        int tmp = strutils::stoi_with_strip(in[0]);

        if(tmp < 0 || tmp > 2)
        {
            std::cout << "procedure type entry out of range\n";
            return;
        }

        bool is_arr = in[2] != "DEP";
        bool is_trans = in[3] != "PROC";
        bool ret = false;
        if(is_trans)
        {
            ret = av->fpl->set_arpt_proc_trans(ProcType(tmp), in[1], is_arr);
        }
        else
        {
            ret = av->fpl->set_arpt_proc(ProcType(tmp), in[1], is_arr);
        }

        if(!ret)
        {
            std::cout << "Failed to set procedure/trantition\n";
        }
    }

    inline void print_legs(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: layout{1/2}\n";
            return;
        }

        bool show_dist_trk = in[0] == "2";

        if(in[0] != "1" && in[0] != "2")
            return;

        size_t n_legs;
        auto legs = av->get_leg_list(&n_legs);

        size_t cnt = 0;
        for(auto i: legs)
        {
            if(cnt && cnt < size_t(n_legs-1))
            {
                std::cout << cnt-1 << ". ";
                if(i.data.is_discon)
                {
                    std::cout << "DISCONTINUITY\n";
                    cnt++;
                    continue;
                }
                double lat_deg = i.data.leg.main_fix.data.pos.lat_rad * geo::RAD_TO_DEG;
                double lon_deg = i.data.leg.main_fix.data.pos.lon_rad * geo::RAD_TO_DEG;
                if(i.data.leg.leg_type != "IF" && show_dist_trk)
                {
                    float brng_deg = i.data.misc_data.true_trk_deg;
                    float dist_nm = i.data.leg.outbd_dist_time;
                    std::string brng_str = strutils::double_to_str(double(brng_deg), 6);
                    std::string dist_str = strutils::double_to_str(double(dist_nm), 6);
                    std::cout << brng_str << " " << dist_str << "\n";
                }
                std::string pos = "";
                if(!show_dist_trk)
                    pos = strutils::double_to_str(lat_deg, 6) + " " + strutils::double_to_str(lon_deg, 6);
                std::string misc_data = i.data.leg.main_fix.id + " " + i.data.leg.leg_type;

                std::cout << misc_data + " " + pos << "\n";
            }
            cnt++;
        }
    }

    inline void add_via(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Command expects 2 arguments: {Next segment index}, {Airway name}\n";
            return;
        }

        size_t idx = size_t(strutils::stoi_with_strip(in[0]));
        size_t n_segs;
        auto segs = av->get_seg_list(&n_segs);

        seg_list_node_t *s_ptr = nullptr;
        if(idx < n_segs)
        {
            s_ptr = segs[idx].ptr;
        }
        double id = av->seg_list_id;
        bool retval = av->fpl->add_enrt_seg({s_ptr, id}, in[1]);

        if(!retval)
        {
            std::cout << "Invalid entry\n";
        }
    }

    inline void delete_via(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: {Next segment index}\n";
            return;
        }

        size_t idx = size_t(strutils::stoi_with_strip(in[0]));
        size_t n_segs;
        auto segs = av->get_seg_list(&n_segs);

        seg_list_node_t *s_ptr = nullptr;
        if(idx < n_segs)
        {
            s_ptr = segs[idx].ptr;
        }
        double id = av->seg_list_id;
        bool retval = av->fpl->delete_via({s_ptr, id});

        if(!retval)
        {
            std::cout << "INVALID DELETE\n";
        }
    }

    inline void add_to(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Command expects 2 arguments: {Next segment index}, {End waypoint name}\n";
            return;
        }

        std::vector<libnav::waypoint_entry_t> wpt_entr;
        size_t n_found = av->navaid_db_ptr->get_wpt_data(in[1], &wpt_entr);

        libnav::waypoint_entry_t tgt;

        if(n_found == 0)
        {
            std::cout << "Invalid waypoint id\n";
        }
        else
        {
            tgt = select_desired(in[1], wpt_entr);
        }

        size_t idx = size_t(strutils::stoi_with_strip(in[0]));
        size_t n_segs;
        auto segs = av->get_seg_list(&n_segs);

        seg_list_node_t *s_ptr = nullptr;
        if(idx < n_segs)
        {
            s_ptr = segs[idx].ptr;
        }
        libnav::waypoint_t tgt_wpt = {in[1], tgt};
        double id = av->seg_list_id;
        bool retval = av->fpl->awy_insert({s_ptr, id}, tgt_wpt.get_awy_id());

        if(!retval)
        {
            std::cout << "Invalid entry\n";
        }
    }

    inline void delete_to(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: {Next segment index}\n";
            return;
        }

        size_t idx = size_t(strutils::stoi_with_strip(in[0]));
        size_t n_segs;
        auto segs = av->get_seg_list(&n_segs);

        seg_list_node_t *s_ptr = nullptr;
        if(idx < segs.size())
        {
            s_ptr = segs[idx].ptr;
        }
        double id = av->seg_list_id;
        bool retval = av->fpl->delete_seg_end({s_ptr, id});

        if(!retval)
        {
            std::cout << "INVALID DELETE\n";
        }
    }

    inline void legs_set(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 3 && in.size() != 4)
        {
            std::cout << "Command expects 3 arguments: {index}, {L/R CDU}, {L/R Field}, (optional){Scratch pad content. If not empty}\n";
            return;
        }
        
        if(in[2] == "R")
            return;

        size_t idx = size_t(strutils::stoi_with_strip(in[0]))+1;
        size_t n_legs;
        auto legs = av->get_leg_list(&n_legs);

        if(idx >= n_legs)
        {
            std::cout << "Index out of range\n";
            return;
        }

        std::pair<size_t, double> *ptr;

        if(in[1] == "L")
        {
            ptr = &av->leg_sel_cdu_l;
        }
        else if(in[1] == "R")
        {
            ptr = &av->leg_sel_cdu_r;
        }
        else
        {
            std::cout << "Invalid second parameter\n";
            return;
        }

        if(av->leg_list_id != ptr->second)
        {
            if(in.size() == 4)
            {
                std::vector<libnav::waypoint_entry_t> wpt_entr;
                size_t n_found = av->navaid_db_ptr->get_wpt_data(in[3], &wpt_entr);

                libnav::waypoint_entry_t tgt;

                if(n_found == 0)
                {
                    std::cout << "Invalid waypoint id\n";
                }
                else
                {
                    tgt = select_desired(in[3], wpt_entr);
                }

                av->fpl->add_direct({in[3], tgt}, {legs[idx].ptr, av->leg_list_id});
            }
            else if(idx < n_legs-1)
            {
                leg_list_node_t *leg_ptr = legs[idx].ptr;
                if(!leg_ptr->data.is_discon)
                {
                    ptr->first = idx;
                    ptr->second = av->leg_list_id;
                }
            }
        }
        else if(idx < n_legs-1)
        {
            size_t from = idx;
            size_t to = ptr->first;
            if(to != from)
            {
                if(from > to)
                {
                    if(from+1 < n_legs)
                        from++;
                    std::swap(from, to);
                }
                else if(from)
                {
                    from--;
                }

                av->fpl->dir_from_to({legs[from].ptr, ptr->second}, 
                    {legs[to].ptr, ptr->second});
                ptr->second = -1;
            }
        }
    }

    inline void delete_leg(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: {leg number}\n";
            return;
        }

        size_t idx = size_t(strutils::stoi_with_strip(in[0]))+1;
        size_t n_legs;
        auto legs = av->get_leg_list(&n_legs);

        if(idx >= n_legs-1)
        {
            std::cout << "Index out of range\n";
            return;
        }

        bool ret = av->fpl->delete_leg({legs[idx].ptr, av->leg_list_id});

        if(!ret)
        {
            std::cout << "INVALID DELETE\n";
        }
    }

    inline void print_seg(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 0)
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        size_t n_segs;
        auto segs = av->get_seg_list(&n_segs);

        for(size_t i = 0; i < n_segs; i++)
        {
            auto curr_sg = segs[i];
            leg_list_node_t *end_leg = curr_sg.data.end;
            std::string end_nm = "";
            if(end_leg != nullptr)
            {
                end_nm = end_leg->data.leg.main_fix.id;
            }
            std::cout << curr_sg.data.name << " " << end_nm << " " 
                << curr_sg.data.seg_type << "\n";
        }
    }

    inline void print_refs(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 0)
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        av->fpl->print_refs();
    }

    inline void help(Avionics *av, std::vector<std::string>& in);

    std::unordered_map<std::string, cmd> cmd_map = {
        {"set", set_var},
        {"print", print},
        {"p", print},
        {"quit", quit},
        {"q", quit},
        {"load", load_fpln},
        {"save", save_fpln},
        {"setfilt", set_filter},
        {"fplinfo", fplinfo},
        {"setdep", set_fpl_dep},
        {"setarr", set_fpl_arr},
        {"setdeprwy", set_dep_rwy},
        {"setarrrwy", set_arr_rwy},
        {"getdeprwys", get_dep_rwys},
        {"getarrrwys", get_arr_rwys},
        {"getproc", get_proc},
        {"setproc", set_proc},
        {"addvia", add_via},
        {"deletevia", delete_via},
        {"addto", add_to},
        {"deleteto", delete_to},
        {"legset", legs_set},
        {"deleteleg", delete_leg},
        {"plegs", print_legs},
        {"pseg", print_seg},
        {"prefs", print_refs},
        {"help", help}
        };

    inline void help(Avionics *av, std::vector<std::string>& in)
    {
        (void)av;
        
        if(in.size() != 0)
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        for(auto i: cmd_map)
        {
            std::cout << i.first << "\n";
        }
    }
}
