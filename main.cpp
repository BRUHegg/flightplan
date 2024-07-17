#include "main_helpers.hpp"
#include "fpln/fpl_cmds.hpp"
#include "displays/ND/nd.hpp"
#include <libnav/common.hpp>

const std::string CMD_FILE_NM = "cmds.txt";


int main()
{
    std::string earth_nav_path = "/home/betatest/Documents/programming/nav-test/test_data/";
	std::string xp_ver = "xp12/";
    
    test::Avionics avncs(earth_nav_path+xp_ver+"apt.dat", earth_nav_path+xp_ver+"777_arpt.dat", 
		earth_nav_path+xp_ver+"777_rnw.dat", earth_nav_path+xp_ver+"earth_fix.dat", 
		earth_nav_path+xp_ver+"earth_nav.dat", 
		earth_nav_path+xp_ver+"earth_awy.dat", 
		earth_nav_path+xp_ver+"earth_hold.dat", earth_nav_path+"CIFP");

	StratosphereAvionics::NDData nd_data(avncs.fpl_sys);
	
	std::cout << "Avionics loaded\n";

	std::vector<std::string> pre_exec = {};

	if(libnav::does_file_exist(CMD_FILE_NM))
	{
		std::ifstream file(CMD_FILE_NM);

		std::string line;
		while(getline(file, line))
		{
			if(line.size() && line[0] != '#')
				pre_exec.push_back(line);
		}
	}

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
				test::FPLSys *ptr = avncs.fpl_sys.get();
				test::cmd_map[cmd_name](ptr, args);
			}
			else
			{
				std::cout << "Invalid command name\n";
			}
		}

		avncs.update();
		nd_data.update();
	}
}
