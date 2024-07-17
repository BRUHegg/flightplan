#include "main_helpers.hpp"
#include "fpln/fpl_cmds.hpp"
#include "displays/ND/nd.hpp"
#include <libnav/common.hpp>
#include <libnav/str_utils.hpp>

const std::string CMD_FILE_NM = "cmds.txt";
const std::string PREFS_FILE_NM = "prefs.txt";

const std::string PREFS_EARTH_PATH = "EPATH";
const std::string PREFS_APT_DIR = "APTDIR";


int main()
{
	std::string earth_nav_path = "";
	std::string apt_dat_dir = "";

	bool write_to_prefs = false;

	if(libnav::does_file_exist(PREFS_FILE_NM))
	{
		std::ifstream file(PREFS_FILE_NM);

		std::string line;
		while(getline(file, line))
		{
			line = strutils::strip(line);
			if(line.size() && line[0] != '#')
			{
				std::vector<std::string> str_split = strutils::str_split(line, ' ', 1);

				if(str_split.size() == 2)
				{
					if(str_split[0] == PREFS_EARTH_PATH)
						earth_nav_path = str_split[1];
					else if(str_split[0] == PREFS_APT_DIR)
						apt_dat_dir = str_split[1];
				}
			}
		}

		file.close();
	}

	if(earth_nav_path == "")
	{
		std::cout << "Please enter path to your Resources/default data directory\n";
		std::getline(std::cin, earth_nav_path);

		write_to_prefs = true;
	}

	if(apt_dat_dir == "")
	{
		std::cout << "Please enter path to the directory containing apt.dat\n";
		std::getline(std::cin, apt_dat_dir);

		write_to_prefs = true;
	}

	if(write_to_prefs)
	{
		std::ofstream out(PREFS_FILE_NM, std::ofstream::out);

		out << PREFS_EARTH_PATH << " " << earth_nav_path << "\n";
		out << PREFS_APT_DIR << " " << apt_dat_dir << "\n";

		out.close();
	}

    //std::string earth_nav_path = "/home/betatest/Documents/programming/nav-test/test_data/";
	//std::string xp_ver = "xp12/";
    
    test::Avionics avncs(apt_dat_dir+"apt.dat", "777_arpt.dat", 
		"777_rnw.dat", earth_nav_path+"earth_fix.dat", 
		earth_nav_path+"earth_nav.dat", 
		earth_nav_path+"earth_awy.dat", 
		earth_nav_path+"earth_hold.dat", earth_nav_path+"CIFP");

	StratosphereAvionics::NDData nd_data(avncs.fpl_sys);
	
	std::cout << "Avionics loaded\n";

	std::vector<std::string> pre_exec = {};

	if(libnav::does_file_exist(CMD_FILE_NM))
	{
		std::ifstream file(CMD_FILE_NM);

		std::string line;
		while(getline(file, line))
		{
			line = strutils::strip(line);
			if(line.size() && line[0] != '#')
				pre_exec.push_back(line);
		}

		file.close();
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
