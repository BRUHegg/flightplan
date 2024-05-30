#include <iostream>
#include <memory>
#include <string>
#include "fpln/flightplan.hpp"

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

        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::HoldDB> hold_db;

        std::shared_ptr<FlightPlan> fpl;

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

            cifp_dir_path = cifp_path;

            ac_lat = def_lat;
            ac_lon = def_lon;

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

            fpl = std::make_shared<FlightPlan>(arpt_db_ptr, navaid_db_ptr, cifp_dir_path);
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
            
            if(lon_valid && lat_valid)
            {
                ac_lat = std::stod(env_vars["ac_lat"]);
                ac_lon = std::stod(env_vars["ac_lon"]);
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

    inline void add_seg(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() < 4)
        {
            std::cout << "Command expects 4 arguments: <seg name>, <seg type>, <next>, <legs>\n";
            return;
        }

        auto seg_mp = av->fpl->get_seg_map();
        fpl_segment_types tp = fpl_segment_types(strutils::stoi_with_strip(in[1]));
        seg_list_node_t *next = seg_mp[in[2]];
        std::vector<int> legs;
        for(size_t i = 3; i < in.size(); i++)
        {
            legs.push_back(strutils::stoi_with_strip(in[i])); //addseg A 2 TAIL 1 2 3
        }
        av->fpl->add_segment(legs, tp, in[0], next);
    }

    inline void delbe(Avionics* av, std::vector<std::string>& in)
    {
        auto leg_mp = av->fpl->get_leg_map();
        int start = strutils::stoi_with_strip(in[0]);
        int end = strutils::stoi_with_strip(in[1]);

        av->fpl->delete_between(leg_mp[start], leg_mp[end]);
    }

    inline void print_seg(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        av->fpl->print_seg();
    }

    inline void print_legs(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        av->fpl->print_legs();
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
        {"addseg", add_seg},
        {"pseg", print_seg},
        {"plegs", print_legs},
        {"dbe", delbe}
        };
}
