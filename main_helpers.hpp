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

        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::HoldDB> hold_db;

        std::shared_ptr<FplnInt> fpl;

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

            fpl = std::make_shared<FplnInt>(arpt_db_ptr, navaid_db_ptr, cifp_dir_path);
        }

        std::vector<list_node_ref_t<fpl_seg_t>> get_seg_list()
        {
            std::vector<list_node_ref_t<fpl_seg_t>> scr_data;
            size_t seg_sz = fpl->get_seg_list_sz();
            seg_list_id = fpl->get_sl_seg(0, seg_sz, &scr_data);

            return scr_data;
        }

        std::vector<list_node_ref_t<leg_list_data_t>> get_legs_list()
        {
            std::vector<list_node_ref_t<leg_list_data_t>> scr_data;
            size_t legs_sz = fpl->get_leg_list_sz();
            leg_list_id = fpl->get_ll_seg(0, legs_sz, &scr_data);

            return scr_data;
        }

        void update()
        {
            update_pos();
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

        std::vector<std::string> rwys = av->fpl->get_dep_rwys();
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
        if(in.size() != 2)
        {
            std::cout << "Command expects 2 arguments: {procedure type}, {DEP/ARR}\n";
            return;
        }

        int tmp = strutils::stoi_with_strip(in[0]);

        if(tmp < 0 || tmp > 2)
        {
            std::cout << "procedure type entry out of range\n";
            return;
        }

        bool is_arr = in[1] != "DEP";

        if(is_arr || in[1] == "DEP")
        {
            std::vector<std::string> procs = av->fpl->get_arpt_proc(ProcType(tmp), is_arr);

            for(auto i: procs)
            {
                std::cout << i << "\n";
            }
        }
    }

    inline void set_proc(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 3)
        {
            std::cout << "Command expects 3 arguments: {procedure type}, {proc name}, {DEP/ARR}\n";
            return;
        }

        int tmp = strutils::stoi_with_strip(in[0]);

        if(tmp < 0 || tmp > 2)
        {
            std::cout << "procedure type entry out of range\n";
            return;
        }

        bool is_arr = in[2] != "DEP";
        bool ret = av->fpl->set_arpt_proc(ProcType(tmp), in[1], is_arr);

        if(!ret)
        {
            std::cout << "Failed to set procedure\n";
        }
    }

    inline void print_legs(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 0)
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        auto legs = av->get_legs_list();

        for(auto i: legs)
        {
            double lat_deg = i.data.leg.main_fix.data.pos.lat_rad * geo::RAD_TO_DEG;
            double lon_deg = i.data.leg.main_fix.data.pos.lon_rad * geo::RAD_TO_DEG;
            std::string pos = strutils::double_to_str(lat_deg, 6) + " " + strutils::double_to_str(lon_deg, 6);
            std::string misc_data = i.data.leg.main_fix.id + " " + i.data.leg.leg_type;

            std::cout << misc_data + " " + pos << "\n";
        }
    }

    inline void print_seg(Avionics *av, std::vector<std::string>& in)
    {
        if(in.size() != 0)
        {
            std::cout << "Command expects 0 arguments\n";
            return;
        }

        auto segs = av->get_seg_list();

        for(auto i: segs)
        {
            std::cout << i.data.name << " " << i.data.seg_type << "\n";
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

    std::unordered_map<std::string, cmd> cmd_map = {
        {"set", set_var},
        {"print", print},
        {"p", print},
        {"quit", quit},
        {"q", quit},
        {"fplinfo", fplinfo},
        {"setdep", set_fpl_dep},
        {"setarr", set_fpl_arr},
        {"setdeprwy", set_dep_rwy},
        {"setarrrwy", set_arr_rwy},
        {"getdeprwys", get_dep_rwys},
        {"getarrrwys", get_arr_rwys},
        {"getproc", get_proc},
        {"setproc", set_proc},
        {"plegs", print_legs},
        {"pseg", print_seg},
        {"prefs", print_refs}
        };
}
