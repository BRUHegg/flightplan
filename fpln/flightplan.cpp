#include "flightplan.hpp"
#include <iostream>


namespace test
{
    // FlightPlan class definitions:
    // Public member functions:

    FlightPlan::FlightPlan(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::string cifp_path): 
        leg_data_stack(N_FPL_LEG_CACHE_SZ), seg_stack(N_FPL_SEG_CACHE_SZ), 
        leg_list(), seg_list()
    {
        departure = nullptr;
        arrival = nullptr;

        arpt_db = apt_db;
        navaid_db = nav_db;

        cifp_dir_path = cifp_path;

        fpl_refs = std::vector<fpl_ref_t>(N_FPL_REF_SZ, EmptyRef);
        fpl_refs[0].ptr = &seg_list.head;
        seg_list.head.data.end = &leg_list.head;
    }

    libnav::DbErr FlightPlan::set_dep(std::string icao)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        libnav::DbErr out = set_arpt(icao, &departure);
        if(arrival != nullptr && departure != nullptr && departure->icao_code == icao
            && out != libnav::DbErr::ERR_NONE)
        {
            delete arrival;
            arrival = nullptr;
        }
        return out;
    }

    std::string FlightPlan::get_dep_icao()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure != nullptr)
            return departure->icao_code;
        return "";
    }

    libnav::DbErr FlightPlan::set_arr(std::string icao)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure == nullptr)
        {
            return libnav::DbErr::ERR_NONE;
        }
        
        return set_arpt(icao, &arrival);
    }

    std::string FlightPlan::get_arr_icao()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(arrival != nullptr)
            return arrival->icao_code;
        return "";
    }

    std::vector<std::string> FlightPlan::get_dep_rwys()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure != nullptr)
        {
            return departure->get_rwys();
        }
        return {};
    }

    std::vector<std::string> FlightPlan::get_arr_rwys()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(arrival != nullptr)
        {
            return arrival->get_rwys();
        }
        return {};
    }

    void FlightPlan::print_seg()
    {
        seg_list_node_t *curr = seg_list.head.next;
        while(curr != &seg_list.tail)
        {
            std::cout << "Segment " << curr->data.name << " " << curr->data.seg_type << "\n";
            std::cout << "End leg: " << curr->data.end->data.leg << "\n";
            curr = curr->next;
        }
    }

    void FlightPlan::print_legs()
    {
        leg_list_node_t *curr = leg_list.head.next;
        while(curr != &leg_list.tail)
        {
            std::cout << curr->data.leg << " ";
            curr = curr->next;
        }
        std::cout << "\n";
    }

    FlightPlan::leg_map_t FlightPlan::get_leg_map()
    {
        leg_map_t out;
        out[-1] = &leg_list.head;
        out[-2] = &leg_list.tail;
        leg_list_node_t *curr = leg_list.head.next;
        while(curr != &leg_list.tail)
        {
            out[curr->data.leg] = curr;
            curr = curr->next;
        }

        return out;
    }

    FlightPlan::seg_map_t FlightPlan::get_seg_map()
    {
        seg_map_t out;
        out["HEAD"] = &seg_list.head;
        out["TAIL"] = &seg_list.tail;
        seg_list_node_t *curr = seg_list.head.next;
        while(curr != &seg_list.tail)
        {
            out[curr->data.name] = curr;
            curr = curr->next;
        }

        return out;
    }

    FlightPlan::~FlightPlan()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        reset_fpln();
        leg_data_stack.destroy();
        seg_stack.destroy();
    }

    // Private member functions:

    void FlightPlan::reset_fpln()
    {
        leg_list.release_all(leg_data_stack.ptr_stack);
        seg_list.release_all(seg_stack.ptr_stack);

        for(size_t i = 0; i < N_FPL_REF_SZ; i++)
        {
            fpl_refs[i].name = "";
            fpl_refs[i].ptr = nullptr;
        }
    }

    libnav::DbErr FlightPlan::set_arpt(std::string icao, libnav::Airport **ptr)
    {
        if(*ptr != nullptr && (*ptr)->icao_code == icao)
        {
            return libnav::DbErr::ERR_NONE;
        }
        libnav::Airport *tmp = new libnav::Airport(icao, arpt_db, 
            navaid_db, cifp_dir_path);
        libnav::DbErr err_cd = tmp->err_code;
        if(err_cd != libnav::DbErr::SUCCESS && err_cd != libnav::DbErr::PARTIAL_LOAD)
        {
            delete tmp;
        }
        else
        {
            if(*ptr != nullptr)
            {
                reset_fpln();
                delete *ptr;
            }

            *ptr = tmp;
        }
        
        return err_cd;
    }

    void FlightPlan::delete_between(leg_list_node_t* start, leg_list_node_t* end)
    {
        start->next = end;
        end->prev = start;
        leg_list_node_t *curr = start->next;
        leg_list_node_t *next = curr->next;

        while (curr != end)
        {
            seg_list_node_t *curr_seg = curr->data.seg;
            if(curr_seg != next->data.seg && curr_seg != start->data.seg)
            {
                if(curr_seg == fpl_refs[curr_seg->data.seg_type].ptr)
                {
                    seg_list_node_t *prev_seg = curr->data.seg->prev;
                    if(prev_seg->data.seg_type != curr_seg->data.seg_type)
                    {
                        fpl_refs[curr_seg->data.seg_type].ptr = nullptr;
                    }
                    else
                    {
                        fpl_refs[curr_seg->data.seg_type].ptr = prev_seg;
                    }
                }
                *curr_seg = EmptySeg;
                seg_list.pop(curr_seg, seg_stack.ptr_stack);
            }
            *curr = EmptyNode;
            leg_list.pop(curr, leg_data_stack.ptr_stack);
            curr = next;
            next = curr->next;
        }
    }

    void FlightPlan::delete_segment(seg_list_node_t *seg)
    {
        leg_list_node_t *start = seg->prev->data.end;
        leg_list_node_t *end = seg->data.end->next;
        delete_between(start, end);
    }

    void FlightPlan::add_segment(std::vector<int>& legs, 
        fpl_segment_types seg_tp, std::string seg_name, seg_list_node_t *next)
    {
        seg_list_node_t *prev = next->prev;
        leg_list_node_t *next_leg = prev->data.end->next;

        seg_list_node_t *seg_add = seg_stack.ptr_stack.top();
        seg_stack.ptr_stack.pop();

        seg_add->data.name = seg_name;
        seg_add->data.seg_type = seg_tp;

        for(size_t i = 0; i < legs.size(); i++)
        {
            leg_list_node_t *leg_add = leg_data_stack.ptr_stack.top();
            leg_data_stack.ptr_stack.pop();
            leg_add->data.seg = seg_add;
            leg_add->data.leg = legs[i];

            leg_list.insert_before(next_leg, leg_add);
        }

        seg_add->data.end = next_leg->prev;

        seg_list.insert_before(next, seg_add);
    }

    void FlightPlan::add_legs(std::vector<int>& legs, 
        fpl_segment_types seg_tp, std::string seg_name)
    {
        //if(departure == nullptr || arrival == nullptr || 
        //    seg_tp == 0 || size_t(seg_tp) >= fpl_refs.size())
        //{
        //    return;
        //}
        if(seg_tp == 0 || size_t(seg_tp) >= fpl_refs.size())
        {
            return;
        }
        //size_t ref_idx = size_t(seg_tp);
        seg_list_node_t *ins_seg;
        if(fpl_refs[seg_tp].ptr != nullptr && seg_tp != FPL_SEG_ENRT)
        {
            seg_list_node_t *curr = fpl_refs[seg_tp].ptr;
            seg_list_node_t *prev = curr->prev;
            ins_seg = curr->next;
            while (curr->data.seg_type == seg_tp)
            {
                delete_segment(curr);
                curr = prev;
                prev = curr->prev;
            }
            fpl_refs[seg_tp].ptr = nullptr;
            fpl_refs[seg_tp].name = seg_name;
        }

        add_segment(legs, seg_tp, seg_name, ins_seg);
        fpl_refs[seg_tp].ptr = ins_seg->prev;
    }
} // namespace test
