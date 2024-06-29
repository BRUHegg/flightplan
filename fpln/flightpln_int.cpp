#include "flightpln_int.hpp"


namespace test
{
    std::string get_appr_rwy(std::string& appr)
    {
        std::string rw;

        for(size_t i = 0; i < appr.size(); i++)
        {
            if(rw.size() < 2 && appr[i] >= '0' && appr[i] <= '9')
            {
                rw.push_back(appr[i]);
            }
            else if(rw.size() && (appr[i] == 'L' || appr[i] == 'R' || appr[i] == 'C'))
            {
                rw.push_back(appr[i]);
                break;
            }
            else if(rw.size())
            {
                break;
            }
        }

        return strutils::normalize_rnw_id(rw);
    }

    // FplnInt member functions:
    // Public functions:

    FplnInt::FplnInt(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::string cifp_path):
        FlightPlan(apt_db, nav_db, cifp_path)
    {
        proc_db.resize(N_PROC_DB_SZ);
    }

    libnav::DbErr FplnInt::set_dep(std::string icao)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        libnav::DbErr out = set_arpt(icao, &departure);
        if(departure != nullptr && departure->icao_code == icao
            && out != libnav::DbErr::ERR_NONE)
        {
            dep_rnw = departure->get_rwy_db();
            proc_db[PROC_TYPE_SID] = departure->get_all_sids();
            proc_db[PROC_TYPE_STAR] = departure->get_all_stars();
            proc_db[PROC_TYPE_APPCH] = departure->get_all_appch();

            // If there is an arrival and departure was changed, clear arrival data
            if(arrival != nullptr)
            {
                arr_rnw.clear();
                proc_db[N_ARR_DB_OFFSET+PROC_TYPE_STAR].clear();
                proc_db[N_ARR_DB_OFFSET+PROC_TYPE_APPCH].clear();
                delete arrival;
                arrival = nullptr;
            }
        }
        
        return out;
    }

    std::string FplnInt::get_dep_icao()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure != nullptr)
            return departure->icao_code;
        return "";
    }

    libnav::DbErr FplnInt::set_arr(std::string icao)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure == nullptr)
        {
            return libnav::DbErr::ERR_NONE;
        }
        
        libnav::DbErr err = set_arpt(icao, &arrival, true);
        if(err != libnav::DbErr::ERR_NONE)
        {
            arr_rwy = "";

            if(arrival != nullptr)
            {
                arr_rnw = arrival->get_rwy_db();
                proc_db[N_ARR_DB_OFFSET+PROC_TYPE_STAR] = arrival->get_all_stars();
                proc_db[N_ARR_DB_OFFSET+PROC_TYPE_APPCH] = arrival->get_all_appch();
            }
        }
        return err;
    }

    std::string FplnInt::get_arr_icao()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(arrival != nullptr)
            return arrival->icao_code;
        return "";
    }

    std::vector<std::string> FplnInt::get_dep_rwys(bool filter_rwy, bool filter_sid)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        std::vector<std::string> out = {};
        
        std::string curr_rwy = fpl_refs[FPL_SEG_DEP_RWY].name;
        if(filter_sid && curr_rwy != "")
        {
            out.push_back(curr_rwy);
        }
        else
        {
            std::string curr_sid = fpl_refs[FPL_SEG_SID].name;
            size_t db_idx = get_proc_db_idx(PROC_TYPE_SID, false);

            for(auto i: dep_rnw)
            {
                if(filter_rwy && curr_sid != "" && 
                    proc_db[db_idx][curr_sid].find(i.first) == proc_db[db_idx][curr_sid].end())
                {
                    continue;
                }
                out.push_back(i.first);
            }
        }
        return out;
    }

    std::vector<std::string> FplnInt::get_arr_rwys()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(arrival != nullptr)
        {
            return arrival->get_rwys();
        }
        return {};
    }

    bool FplnInt::set_dep_rwy(std::string& rwy)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(dep_rnw.find(rwy) != dep_rnw.end())
        {
            std::string curr_rwy = fpl_refs[FPL_SEG_DEP_RWY].name;
            if(rwy != curr_rwy)
            {
                std::string curr_sid = fpl_refs[FPL_SEG_SID].name;
                std::string curr_trans = fpl_refs[FPL_SEG_SID_TRANS].name;
                delete_ref(FPL_SEG_SID_TRANS);
                delete_ref(FPL_SEG_SID);

                libnav::arinc_rwy_data_t rwy_data = dep_rnw[rwy];

                leg_t ins_leg;
                ins_leg.leg_type = "IF";
                ins_leg.main_fix.id = rwy;
                ins_leg.main_fix.data.pos = rwy_data.pos;

                std::vector<leg_t> legs = {};

                add_legs(ins_leg, legs, FPL_SEG_DEP_RWY, rwy);
                fpl_refs[FPL_SEG_DEP_RWY].name = rwy;

                set_sid_star(curr_sid);
                set_proc_trans(PROC_TYPE_SID, curr_trans, false);
            }

            return true;
        }

        return false;
    }

    std::string FplnInt::get_dep_rwy()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure != nullptr && fpl_refs[FPL_SEG_DEP_RWY].ptr != nullptr)
            return fpl_refs[FPL_SEG_DEP_RWY].name;
        return "";
    }

    bool FplnInt::set_arr_rwy(std::string& rwy)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(arr_rnw.find(rwy) != arr_rnw.end())
        {
            arr_rwy = rwy;

            return true;
        }

        return false;
    }

    std::string FplnInt::get_arr_rwy()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        return arr_rwy;
    }

    std::vector<std::string> FplnInt::get_arpt_proc(ProcType tp, bool is_arr,
        bool filter_rwy, bool filter_proc)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        fpl_segment_types s_tp = get_proc_tp(tp);
        size_t tp_idx = size_t(s_tp);

        if(filter_rwy && fpl_refs[tp_idx].name != "")
        {
            return {fpl_refs[tp_idx].name};
        }

        size_t db_idx = get_proc_db_idx(tp, is_arr);

        if(db_idx != N_PROC_DB_SZ)
        {
            std::string rwy = "";

            if(filter_proc)
            {
                if(is_arr)
                {
                    rwy = arr_rwy;
                }
                else
                {
                    rwy = fpl_refs[FPL_SEG_DEP_RWY].name;
                }
            }

            return get_proc(proc_db[db_idx], rwy, tp == PROC_TYPE_APPCH);
        }
        
        return {};
    }

    std::vector<std::string> FplnInt::get_arpt_proc_trans(ProcType tp, bool is_rwy, bool is_arr)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        size_t ref_idx = size_t(get_proc_tp(tp));
        std::string proc_name = fpl_refs[ref_idx].name;
        if(proc_name != "")
        {
            size_t db_idx = get_proc_db_idx(tp, is_arr);
            if(is_arr)
            {
                return get_proc_trans(proc_name, proc_db[db_idx], arr_rnw, is_rwy);
            }
            return get_proc_trans(proc_name, proc_db[db_idx], dep_rnw, is_rwy);
        }
        return {};
    }

    bool FplnInt::set_arpt_proc(ProcType tp, std::string proc_nm, bool is_arr)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        size_t db_idx = get_proc_db_idx(tp, is_arr);

        switch(db_idx)
        {
        case PROC_TYPE_SID:
            return set_sid_star(proc_nm);
        case PROC_TYPE_STAR+N_ARR_DB_OFFSET:
            return set_sid_star(proc_nm, true);
        case PROC_TYPE_APPCH+N_ARR_DB_OFFSET:
            return set_appch(proc_nm);
        default:
            return false;
        }

        return false;
    }

    bool FplnInt::set_arpt_proc_trans(ProcType tp, std::string trans, bool is_arr)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        return set_proc_trans(tp, trans, is_arr);
    }

    // Private functions:

    size_t FplnInt::get_proc_db_idx(ProcType tp, bool is_arr)
    {
        if(tp == PROC_TYPE_SID && is_arr)
        {
            return N_PROC_DB_SZ;
        }

        return tp + N_ARR_DB_OFFSET * is_arr;
    }

    fpl_segment_types FplnInt::get_proc_tp(ProcType tp)
    {
        switch (tp)
        {
        case PROC_TYPE_SID:
            return FPL_SEG_SID;
        case PROC_TYPE_STAR:
            return FPL_SEG_STAR;
        case PROC_TYPE_APPCH:
            return FPL_SEG_APPCH;
        default:
            return FPL_SEG_NONE;
        }
    }

    fpl_segment_types FplnInt::get_trans_tp(ProcType tp)
    {
        switch (tp)
        {
        case PROC_TYPE_SID:
            return FPL_SEG_SID_TRANS;
        case PROC_TYPE_STAR:
            return FPL_SEG_STAR_TRANS;
        case PROC_TYPE_APPCH:
            return FPL_SEG_APPCH_TRANS;
        default:
            return FPL_SEG_NONE;
        }
    }

    std::vector<std::string> FplnInt::get_proc(libnav::str_umap_t& db, std::string rw, bool is_appch)
    {
        std::vector<std::string> out;

        for(auto i: db)
        {
            if(rw != "" && i.second.find(rw) == i.second.end() && !is_appch)
            {
                continue;
            }
            else if(is_appch && rw != "")
            {
                std::string curr_nm = i.first;
                std::string c_rnw = get_appr_rwy(curr_nm);
                if(c_rnw != rw)
                {
                    continue;
                }
            }
            out.push_back(i.first);
        }

        return out;
    }

    std::vector<std::string> FplnInt::get_proc_trans(std::string proc, libnav::str_umap_t& db, 
        libnav::arinc_rwy_db_t& rwy_db, bool is_rwy)
    {
        std::vector<std::string> out;

        assert(db.find(proc) != db.end());

        for(auto i: db[proc])
        {
            if(is_rwy && rwy_db.find(i) != rwy_db.end())
            {
                out.push_back(i);
            }
            else if(!is_rwy && rwy_db.find(i) == rwy_db.end())
            {
                out.push_back(i);
            }
        }

        return out;
    }

    bool FplnInt::add_fpl_seg(libnav::arinc_leg_seq_t& legs, fpl_segment_types seg_tp, std::string ref_nm)
    {
        if(legs.size())
        {
            size_t seg_idx = size_t(seg_tp);
            leg_t start = legs[0];
            std::vector<leg_t> legs_ins;

            for(size_t i = 1; i < legs.size(); i++)
            {
                legs_ins.push_back(legs[i]);
            }

            add_legs(start, legs_ins, seg_tp, ref_nm);
            fpl_refs[seg_idx].name = ref_nm;

            return true;
        }

        return false;
    }

    bool FplnInt::set_sid_star(std::string proc_nm, bool is_star)
    {
        size_t db_idx;
        ProcType proc_tp;
        fpl_segment_types proc_seg, trans_seg;
        if(!is_star) 
        {
            proc_tp = PROC_TYPE_SID;
            db_idx = get_proc_db_idx(PROC_TYPE_SID, false);
            proc_seg = FPL_SEG_SID;
            trans_seg = FPL_SEG_SID_TRANS;
        }
        else
        {
            proc_tp = PROC_TYPE_STAR;
            db_idx = get_proc_db_idx(PROC_TYPE_STAR, true);
            proc_seg = FPL_SEG_STAR;
            trans_seg = FPL_SEG_STAR_TRANS;
        }

        if(proc_db[db_idx].find(proc_nm) != proc_db[db_idx].end())
        {
            std::string rwy;
            if(!is_star)
                rwy = fpl_refs[FPL_SEG_DEP_RWY].name;
            else
                rwy = arr_rwy;

            if(rwy == "")
            {
                delete_ref(trans_seg);
                delete_ref(proc_seg);
                fpl_refs[size_t(proc_seg)].name = proc_nm;
            }
            else
            {
                libnav::arinc_leg_seq_t legs;
                
                if(!is_star)
                    legs = departure->get_sid(proc_nm, rwy);
                else
                    legs = arrival->get_star(proc_nm, rwy);

                std::string trans_nm = fpl_refs[size_t(trans_seg)].name;
                delete_ref(trans_seg);
                bool retval = add_fpl_seg(legs, proc_seg, proc_nm);
                if(!retval) // Case: runway doesn't belong to sid
                {
                    delete_ref(proc_seg);
                }

                set_proc_trans(proc_tp, trans_nm, is_star);
                
                return retval;
            }
        }

        return false;
    }

    bool FplnInt::set_appch(std::string appch)
    {
        size_t db_idx = get_proc_db_idx(PROC_TYPE_APPCH, true);
        if(proc_db[db_idx].find(appch) != proc_db[db_idx].end())
        {
            std::string tmp = NONE_TRANS;
            libnav::arinc_leg_seq_t legs = arrival->get_appch(appch, tmp);
            std::string curr_tr = fpl_refs[FPL_SEG_APPCH].name;
            delete_ref(FPL_SEG_APPCH_TRANS);

            bool added = add_fpl_seg(legs, FPL_SEG_APPCH, appch);
            if(added)
            {
                arr_rwy = get_appr_rwy(appch);
                set_proc_trans(PROC_TYPE_APPCH, curr_tr, true);
                return true;
            }
        }

        arr_rwy = "";
        delete_ref(FPL_SEG_APPCH_TRANS);
        delete_ref(FPL_SEG_APPCH);
        return false;
    }

    bool FplnInt::set_proc_trans(ProcType tp, std::string trans, bool is_arr)
    {
        //if(trans == "NONE")
        //{
        //    trans = "";
        //}

        size_t db_idx = get_proc_db_idx(tp, is_arr);
        size_t seg_tp = size_t(get_proc_tp(tp));
        fpl_segment_types t_tp = get_trans_tp(tp);
        size_t t_idx = size_t(t_tp);

        std::string curr_proc = fpl_refs[seg_tp].name;
        
        if(curr_proc != "" && fpl_refs[seg_tp].ptr == nullptr && 
            proc_db[db_idx][curr_proc].find(trans) != proc_db[db_idx][curr_proc].end())
        {
            delete_ref(t_tp);
            fpl_refs[t_idx].name = trans;
            return false;
        }
        else if(curr_proc == "")
        {
            delete_ref(t_tp);
            return false;
        }
        libnav::Airport *apt = departure;
        if(is_arr)
        {
            apt = arrival;
        }

        libnav::arinc_leg_seq_t legs = {};

        if(tp == PROC_TYPE_SID)
        {
            legs = apt->get_sid(curr_proc, trans);
        }
        else if(tp == PROC_TYPE_STAR)
        {
            legs = apt->get_star(curr_proc, trans);
        }
        else if(tp == PROC_TYPE_APPCH)
        {
            legs = apt->get_appch(curr_proc, trans);
        }
        
        bool added = add_fpl_seg(legs, t_tp, trans);
        if(!added)
        {
            delete_ref(t_tp);
        }
        else
        {
            return true;
        }

        return false;
    }
} // namespace test
