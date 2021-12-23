#include "beliefstate.h"
#include "simulator.h"
#include "simulator_lazy.h"
#include "utils.h"

using namespace UTILS;

BELIEF_STATE::BELIEF_STATE(): metainfo(nullptr)
{
    Samples.clear();
}

void BELIEF_STATE::Free(const SIMULATOR& simulator)
{
    for (std::vector<STATE*>::iterator i_state = Samples.begin();
            i_state != Samples.end(); ++i_state)
    {
        simulator.FreeState(*i_state);
    }
    Samples.clear();

    if (metainfo) {
        simulator.FreeMetainfo(metainfo);
        metainfo = nullptr;
    }
}

void BELIEF_STATE::Free(const SIMULATOR_LAZY& simulator)
{
    for (std::vector<STATE*>::iterator i_state = Samples.begin();
            i_state != Samples.end(); ++i_state)
    {
        simulator.FreeState(*i_state);
    }
    Samples.clear();

    if (metainfo) {
        simulator.FreeMetainfo(metainfo);
        metainfo = nullptr;
    }
}

STATE* BELIEF_STATE::CreateSample(const SIMULATOR& simulator) const
{
    int index = Random(Samples.size());
    return simulator.Copy(*Samples[index]);
}

STATE* BELIEF_STATE::CreateSample(const SIMULATOR_LAZY& simulator) const
{
    int index = Random(Samples.size());
    return simulator.Copy(*Samples[index]);
}

void BELIEF_STATE::AddSample(STATE* state)
{
    Samples.push_back(state);
    if (metainfo)
        metainfo->update(state);
}

void BELIEF_STATE::Copy(const BELIEF_STATE& beliefs, const SIMULATOR& simulator)
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        AddSample(simulator.Copy(**i_state));
    }

    // delete previous
    if (metainfo) {
        simulator.FreeMetainfo(metainfo);
        metainfo = nullptr;
    }
    // copy new, if any
    if (beliefs.metainfo)
        metainfo = beliefs.get_metainfo().clone();
    else
        metainfo = nullptr;
}

void BELIEF_STATE::Copy(const BELIEF_STATE& beliefs, const SIMULATOR_LAZY& simulator)
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        AddSample(simulator.Copy(**i_state));
    }

    // delete previous
    if (metainfo) {
        simulator.FreeMetainfo(metainfo);
        metainfo = nullptr;
    }
    // copy new, if any
    if (beliefs.metainfo)
        metainfo = beliefs.get_metainfo().clone();
    else
        metainfo = nullptr;
}

void BELIEF_STATE::set_metainfo(const BELIEF_META_INFO &m, const SIMULATOR& simulator) {
    if (metainfo) {
        simulator.FreeMetainfo(metainfo);
        metainfo = nullptr;
    }
    metainfo = m.clone();
}

void BELIEF_STATE::set_metainfo(const BELIEF_META_INFO &m, const SIMULATOR_LAZY& simulator) {
    if (metainfo) {
        simulator.FreeMetainfo(metainfo);
        metainfo = nullptr;
    }
    metainfo = m.clone();
}

void BELIEF_STATE::Move(BELIEF_STATE& beliefs)
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        AddSample(*i_state);
    }
    beliefs.Samples.clear();

    metainfo = beliefs.metainfo;
    beliefs.metainfo = nullptr;
}

