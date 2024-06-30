#include "main_helpers.hpp"


int main()
{
    std::string earth_nav_path = "C:/Users/bieden/Documents/test_data/";
	std::string xp_ver = "xp12/";
    
    test::Avionics avncs(earth_nav_path+xp_ver+"apt.dat", earth_nav_path+xp_ver+"777_arpt.dat", 
		earth_nav_path+xp_ver+"777_rnw.dat", earth_nav_path+xp_ver+"earth_fix.dat", 
		earth_nav_path+xp_ver+"earth_nav.dat", 
		earth_nav_path+xp_ver+"earth_awy.dat", 
		earth_nav_path+xp_ver+"earth_hold.dat", earth_nav_path+"CIFP");
	std::cout << "Avionics loaded\n";

	std::vector<std::string> pre_exec = {
		"setdep KSEA",
		"setarr KSEA",
		"setdeprwy 34R",
		"setproc 0 ISBRG1 DEP PROC",
		"setproc 1 JAWBN6 ARR PROC",
		"setproc 2 ILS34C ARR PROC",
		"plegs"
		//"setdeprwy 28L",
		//"fplinfo",
		//"getproc 0 DEP"
	};
	//pre_exec = {};

	size_t i = 0;

    while(1)
	{
		std::string in_raw;
		if(i < pre_exec.size())
		{
			in_raw = pre_exec[i];
			i++;
		}
		else
		{
			std::cout << ">> ";
			std::getline(std::cin, in_raw);
		}
		
		
		std::string in_proc = strutils::strip(in_raw, ' ');

		std::vector<std::string> line_split = strutils::str_split(in_proc, ' ');
		if(line_split.size())
		{
			std::string cmd_name = line_split[0];
			std::vector<std::string> args = std::vector<std::string>(line_split.begin() + 1, 
				line_split.end());

			if(test::cmd_map.find(cmd_name) != test::cmd_map.end())
			{
				test::cmd_map[cmd_name](&avncs, args);
			}
			else
			{
				std::cout << "Invalid command name\n";
			}
		}

		avncs.update();
	}
}
