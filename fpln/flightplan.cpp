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
        seg_list.tail.data.end = &leg_list.tail;

        seg_list.head.data.seg_type = FPL_SEG_NONE;
        seg_list.tail.data.seg_type = FPL_SEG_NONE;
        
        leg_list.head.data.seg = &seg_list.head;
        leg_list.tail.data.seg = &seg_list.tail;
    }

    size_t FlightPlan::get_leg_list_sz()
    {
        return leg_list.size;
    }

    size_t FlightPlan::get_seg_list_sz()
    {
        return seg_list.size;
    }

    double FlightPlan::get_ll_seg(size_t start, size_t l, 
            std::vector<list_node_ref_t<leg_list_data_t>>* out)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(start >= leg_list.size)
        {
            return -1;
        }
        size_t i = 0;
        leg_list_node_t* curr = &(leg_list.head);
        while(i != start)
        {
            curr = curr->next;
            i++;
        }

        while(l && i < leg_list.size)
        {
            out->push_back({curr, curr->data});
            l--;
            curr = curr->next;
        }

        return leg_list.id;
    }

    double FlightPlan::get_sl_seg(size_t start, size_t l, 
            std::vector<list_node_ref_t<fpl_seg_t>>* out)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(start >= seg_list.size)
        {
            return -1;
        }
        size_t i = 0;
        seg_list_node_t* curr = &(seg_list.head);
        while(i != start)
        {
            curr = curr->next;
            i++;
        }

        while(l && i < seg_list.size)
        {
            out->push_back({curr, curr->data});
            l--;
            curr = curr->next;
        }

        return seg_list.id;
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

    void FlightPlan::print_refs()
    {
        for(size_t i = 1; i < fpl_refs.size(); i++)
        {
            std::cout << seg_to_str[fpl_segment_types(i)] << " ";
            seg_list_node_t *curr = fpl_refs[i].ptr;
            if(curr != nullptr)
            {
                std::cout << "Segment " << curr->data.name << " " << curr->data.seg_type << "\n";
                std::cout << "End leg: " << curr->data.end->data.leg << "\n";
            }
            else
            {
                std::cout << "\n";
            }
        }
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
                    //if(prev_seg != &(seg_list.head) && 
                    //    prev_seg->data.is_discon)
                    //{
                    //    prev_seg = prev_seg->prev;
                    //}
                    if(prev_seg->data.seg_type != curr_seg->data.seg_type)
                    {
                        fpl_refs[curr_seg->data.seg_type].ptr = nullptr;
                    }
                    else
                    {
                        fpl_refs[curr_seg->data.seg_type].ptr = prev_seg;
                    }
                }
                
                seg_list.pop(curr_seg, seg_stack.ptr_stack);
                *curr_seg = EmptySeg;
            }
            else if(curr_seg != next->data.seg)
            {
                curr_seg->data.end = start;
            }
            
            leg_list.pop(curr, leg_data_stack.ptr_stack);
            *curr = EmptyNode;
            curr = next;
            next = curr->next;
        }
    }

    void FlightPlan::delete_segment(seg_list_node_t *seg,bool leave_seg)
    {
        leg_list_node_t *start = seg->prev->data.end;
        leg_list_node_t *end;
        seg_list_node_t *next_seg = seg->next;
        if(next_seg != &(seg_list.tail) && !next_seg->data.is_direct && 
            !next_seg->data.is_discon && leave_seg)
        {
            end = seg->data.end;
            seg->data.is_direct = true;
            seg->data.name = DCT_LEG_NAME;
        }
        else
        {
            end = seg->data.end->next;
        }
        delete_between(start, end);
    }

    void FlightPlan::add_segment(std::vector<int>& legs, 
        fpl_segment_types seg_tp, std::string seg_name, seg_list_node_t *next, 
        bool is_direct)
    {
        if(!legs.size())
            return;
        seg_list_node_t *prev = next->prev;
        leg_list_node_t *next_leg = prev->data.end->next;

        seg_list_node_t *seg_add = seg_stack.get_new();
        if(seg_add != nullptr)
        {
            seg_add->data.name = seg_name;
            seg_add->data.seg_type = seg_tp;
            seg_add->data.is_direct = is_direct;

            for(size_t i = 0; i < legs.size(); i++)
            {
                leg_list_data_t c_data;
                c_data.seg = seg_add;
                c_data.leg = legs[i];
                c_data.is_discon = false;
                add_singl_leg(next_leg, c_data);
            }

            seg_add->data.end = next_leg->prev;
            seg_list.insert_before(next, seg_add);
        }
    }

    void FlightPlan::add_discon(seg_list_node_t *next)
    {
        seg_list_node_t *prev = next->prev;
        leg_list_node_t *next_leg = prev->data.end->next;

        seg_list_node_t *seg_add = seg_stack.get_new();
        if(seg_add != nullptr)
        {
            seg_add->data.name = DISCON_SEG_NAME;
            seg_add->data.is_discon = true;
            fpl_segment_types prev_seg_tp = prev->data.seg_type;
            seg_add->data.seg_type = prev_seg_tp;
            leg_list_data_t c_data;
            c_data.seg = seg_add;
            c_data.is_discon = true;
            c_data.leg = -1;
            add_singl_leg(next_leg, c_data);

            seg_add->data.end = next_leg->prev;
            seg_list.insert_before(next, seg_add);

            if(fpl_refs[size_t(prev_seg_tp)].ptr == prev)
            {
                fpl_refs[size_t(prev_seg_tp)].ptr = seg_add;
            }
        }
    }

    void FlightPlan::add_legs(int start, std::vector<int>& legs, 
        fpl_segment_types seg_tp, std::string seg_name, seg_list_node_t *next)
    {
        if(seg_tp == 0 || size_t(seg_tp) >= fpl_refs.size())
        {
            return;
        }
        
        seg_list_node_t *ins_seg;  // Segment before which the legs are to be inserted
        seg_list_node_t *next_seg;  // Segment at or before insert segment. It's one past the end of the segment sequence
        if(next == nullptr)
        {
            ins_seg = get_insert_seg(seg_tp, &next_seg);
        }
        else
        {
            ins_seg = next;
            next_seg = ins_seg;
            seg_tp = ins_seg->prev->data.seg_type;
        }

        std::vector<int> legs_add;
        if(ins_seg->prev != &(seg_list.head) && 
            ins_seg->prev->data.seg_type != FPL_SEG_DEP_RWY)
        {
            std::vector<int> vec = {start};
            seg_list_node_t *tmp_seg = ins_seg->prev;

            if(tmp_seg->data.is_discon)
                tmp_seg = tmp_seg->prev;

            add_segment(vec, seg_tp, DCT_LEG_NAME, ins_seg, true);
            
            if(tmp_seg != &(seg_list.head))
                merge_seg(tmp_seg);

            legs_add = legs;
        }
        else
        {
            legs_add.push_back(start);
            for(auto i: legs)
            {
                legs_add.push_back(i);
            }
        }
        add_segment(legs_add, seg_tp, seg_name, ins_seg);
        merge_seg(ins_seg->prev);
        fpl_refs[seg_tp].ptr = next_seg->prev;
    }

    void FlightPlan::add_singl_leg(leg_list_node_t *next, leg_list_data_t data)
    {
        leg_list_node_t *leg_add = leg_data_stack.get_new();
        if(leg_add != nullptr)
        {
            leg_add->data = data;

            leg_list.insert_before(next, leg_add);
        }
    }

    void FlightPlan::add_direct(int leg, leg_list_node_t *next_leg)
    {
        if(fpl_refs[size_t(FPL_SEG_DEP_RWY)].ptr == nullptr)
            return;
        leg_list_node_t *prev_leg = next_leg->prev;

        seg_list_node_t *prev_seg = prev_leg->data.seg;
        seg_list_node_t *next_seg = next_leg->data.seg;

        std::vector<int> legs_add = {leg};

        fpl_segment_types dir_tp = next_seg->data.seg_type;
        if(prev_seg->data.seg_type > dir_tp)
            dir_tp = prev_seg->data.seg_type;

        if(next_leg != &leg_list.tail)
            next_seg = subdivide(prev_leg, next_leg);

        if(next_seg != nullptr && prev_leg->data.leg != leg && next_leg->data.leg != leg)
        {
            add_segment(legs_add, dir_tp, DCT_LEG_NAME, next_seg, true);

            if(next_leg != &leg_list.tail && !next_leg->data.is_discon)
                add_discon(next_seg);
        }
    }

    void FlightPlan::delete_leg(leg_list_node_t *leg)
    {
        leg_list_node_t *prev_leg = leg->prev;
        leg_list_node_t *next_leg = leg->next;

        seg_list_node_t *leg_seg = leg->data.seg;

        // Case: the leg is at the end of the flightplan
        if(next_leg == &leg_list.tail)
        {
            int dist_l = 0;
            leg_list_node_t *prev_check = prev_leg;

            while (prev_check->data.seg == leg->data.seg && dist_l < 2)
            {
                dist_l++;
                prev_check = prev_check->prev;
            }
            
            if(dist_l == 0)
            {
                delete_segment(leg_seg, false);
                return;
            }
            else
            {
                delete_between(prev_leg, next_leg);
                if(dist_l == 1)
                    leg_seg->data.is_direct = true;
            }
        }
        else if(leg_seg->data.is_direct)
        {
            leg_seg->data.is_direct = false;
            leg_seg->data.is_discon = true;
            leg_seg->data.name = DISCON_SEG_NAME;
            leg_seg->data.end->data.is_discon = true;
            leg_seg->data.end->data.leg = -1;
        }
        else
        {
            seg_list_node_t *seg_add = seg_stack.get_new();
            seg_list_node_t *seg_next = subdivide(prev_leg, next_leg);
            if(seg_add != nullptr && seg_next != nullptr)
            {
                seg_add->data.name = DISCON_SEG_NAME;
                seg_add->data.is_direct = false;
                seg_add->data.is_discon = true;
                seg_add->data.seg_type = leg_seg->data.seg_type;
                seg_add->data.end = leg;
                
                seg_list.insert_before(seg_next, seg_add);
            }
            else if(seg_add != nullptr)
            {
                seg_stack.ptr_stack.push(seg_add);
            }
        }
    }

    seg_list_node_t *FlightPlan::get_insert_seg(fpl_segment_types seg_tp, 
            seg_list_node_t **next_seg)
    {
        size_t ref_idx = size_t(seg_tp);
        seg_list_node_t *ins_seg;
        if(fpl_refs[ref_idx].ptr != nullptr && seg_tp != FPL_SEG_ENRT)
        {
            seg_list_node_t *curr = fpl_refs[ref_idx].ptr;
            seg_list_node_t *prev = curr->prev;
            ins_seg = curr->next;
            *next_seg = ins_seg;
            while (curr->data.seg_type == seg_tp || curr->data.is_discon)
            {
                delete_segment(curr);
                curr = prev;
                prev = curr->prev;
            }
            while(ins_seg->prev->data.seg_type == seg_tp)
            {
                ins_seg = ins_seg->prev;
            }
            fpl_refs[ref_idx].ptr = nullptr;
        }
        else if(fpl_refs[ref_idx].ptr == nullptr)
        {
            while(ref_idx > 0)
            {
                ref_idx--;
                if(fpl_refs[ref_idx].ptr != nullptr)
                {
                    ins_seg = fpl_refs[ref_idx].ptr->next;
                    break;
                }
            }
            *next_seg = ins_seg;
        }
        else
        {
            ins_seg = fpl_refs[ref_idx].ptr->next;
            *next_seg = ins_seg;
        }

        return ins_seg;
    }

    void FlightPlan::merge_seg(seg_list_node_t *tgt)
    {
        int i = 1;
        seg_list_node_t *curr = tgt->next;
        seg_list_node_t *next_disc = nullptr;  // Next discountinuity segment
        seg_list_node_t *next_dir = nullptr;  // Next "direct to" segment
        while(i+1 && curr != &(seg_list.tail))
        {
            if(i == 1 && curr->data.is_discon)
            {
                next_disc = curr;
            }
            else
            {
                if(curr->data.is_direct)
                    next_dir = curr;
                break;
            }
            curr = curr->next;
            i--;
        }
        if(next_dir != nullptr)
        {
            leg_list_node_t *tgt_leg = tgt->data.end;
            leg_list_node_t *dct_leg = next_dir->data.end;

            if(tgt_leg->data.leg == dct_leg->data.leg)
            {
                delete_segment(next_dir, false);
                if(next_disc != nullptr)
                {
                    delete_segment(next_disc, false);
                }
            }
            else if(next_disc == nullptr)
            {
                add_discon(curr);
            }
        }
    }

    seg_list_node_t *FlightPlan::subdivide(leg_list_node_t *prev_leg, leg_list_node_t *next_leg)
    {
        leg_list_node_t *prev_check = prev_leg;
        leg_list_node_t *next_check = next_leg;

        seg_list_node_t *prev_seg = prev_leg->data.seg;
        seg_list_node_t *next_seg = next_leg->data.seg;

        int dist_l = 0;
        int dist_r = 0;

        while(prev_check->data.seg == next_leg->data.seg && dist_l < 2)
        {
            prev_check = prev_check->prev;
            dist_l++;
        }

        while(next_check->data.seg == prev_leg->data.seg && dist_r < 2)
        {
            next_check = next_check->next;
            dist_r++;
        }

        if(dist_l != 0)
        {
            // Divide existing segment into 2
            seg_list_node_t *seg_add = seg_stack.get_new();
            if(seg_add != nullptr)
            {
                seg_add->data = prev_seg->data;
                seg_add->data.end = prev_leg;

                if(dist_l == 1)
                {
                    seg_add->data.is_direct = true;
                    seg_add->data.name = DCT_LEG_NAME;
                }

                if(dist_r == 1)
                {
                    prev_seg->data.is_direct = true;
                    prev_seg->data.name = DCT_LEG_NAME;
                }  
                
                seg_list.insert_before(prev_seg, seg_add);
                next_seg = prev_seg;
            }
        }
        
        // add start of the 2nd subsegment as a direct
        if(next_seg != &seg_list.tail && !next_seg->data.is_direct && 
            !next_seg->data.is_discon)
        {
            seg_list_node_t *seg_add = seg_stack.get_new();
            if(seg_add != nullptr)
            {
                seg_add->data.is_direct = true;
                seg_add->data.is_discon = false;
                seg_add->data.name = DCT_LEG_NAME;
                seg_add->data.seg_type = next_seg->data.seg_type;
                seg_add->data.end = next_leg;

                seg_list.insert_before(next_seg, seg_add);
            }

            return seg_add;
        }

        return next_seg;
    }
} // namespace test
