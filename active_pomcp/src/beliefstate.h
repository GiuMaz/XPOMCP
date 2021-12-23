#ifndef BELIEF_STATE_H
#define BELIEF_STATE_H

#include <vector>

class STATE;
class SIMULATOR;
class SIMULATOR_LAZY;

class BELIEF_META_INFO {
public:
    virtual void update(STATE *) {}
    virtual void clear() {}
    virtual BELIEF_META_INFO *clone() const {
        return nullptr;
    }
};

class BELIEF_STATE
{
public:
    BELIEF_STATE();

    //BELIEF_STATE(const BELIEF_STATE &b) = default;

    // Free memory for all states
    void Free(const SIMULATOR& simulator);
    void Free(const SIMULATOR_LAZY& simulator);

    // Creates new state, now owned by caller
    STATE* CreateSample(const SIMULATOR& simulator) const;
    STATE* CreateSample(const SIMULATOR_LAZY& simulator) const;

    // Added state is owned by belief state
    void AddSample(STATE* state);

    // Make own copies of all samples
    void Copy(const BELIEF_STATE& beliefs, const SIMULATOR& simulator);
    void Copy(const BELIEF_STATE& beliefs, const SIMULATOR_LAZY& simulator);

    // Move all samples into this belief state
    void Move(BELIEF_STATE& beliefs);

    bool Empty() const { return Samples.empty(); }
    int GetNumSamples() const { return Samples.size(); }
    const STATE* GetSample(int index) const { return Samples[index]; }

    BELIEF_META_INFO &get_metainfo() {
        return *metainfo;
    }

    const BELIEF_META_INFO &get_metainfo() const {
        return *metainfo;
    }

    void set_metainfo(const BELIEF_META_INFO &m, const SIMULATOR& simulator);
    void set_metainfo(const BELIEF_META_INFO &m, const SIMULATOR_LAZY& simulator);

private:

    std::vector<STATE*> Samples;
    BELIEF_META_INFO *metainfo;
};

#endif // BELIEF_STATE_H
