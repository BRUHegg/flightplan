#include <iostream>
#include <memory>
#include <string>
#include "fpln/fpln_sys.hpp"
#include <libnav/geo_utils.hpp>


namespace test
{
    class Avionics
    {
    public:
        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::HoldDB> hold_db;

        std::shared_ptr<FPLSys> fpl_sys;

        std::string cifp_dir_path;


        Avionics(std::string apt_dat, std::string custom_apt, std::string custom_rnw,
            std::string fix_data, std::string navaid_data, std::string awy_data,
            std::string hold_data, 
            std::string cifp_path)
        {

            cifp_dir_path = cifp_path;

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

            fpl_sys = std::make_shared<FPLSys>(arpt_db_ptr, navaid_db_ptr, awy_db, 
                cifp_dir_path);
        }

        void update()
        {
            fpl_sys->update();
        }

        ~Avionics()
        {
            fpl_sys.reset();
            hold_db.reset();
            awy_db.reset();
            navaid_db_ptr.reset();
            navaid_db_ptr.reset();
            arpt_db_ptr.reset();
        }
    };


    //class CMDInterface
    //{
    //public:
    //    CMDInterface()
    //    {
    //
    //    }
    //
    //private:
    //    std::shared_ptr<Avionics> avncs;
    //};
}
