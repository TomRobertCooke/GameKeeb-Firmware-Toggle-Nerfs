#ifndef _MODES_MELEELIMITS_HPP
#define _MODES_MELEELIMITS_HPP

#include "nerfs/core/socd.hpp"
#include "nerfs/core/state.hpp"

#include "inputs.hpp"
#include "gamecube_definitions.h"

// override the socd assigned, just for melee configurations
#define MELEE_SOCD socd::SOCD_NEUTRAL

//#define MELEE_SOCD socd::SOCD_2IP_NO_REAC

enum abtest {
    AB_A,
    AB_B
};

void limitOutputs(
    const uint16_t sampleSpacing,
    const abtest whichAB,
    const InputState &inputs,
    const OutputState &rawOutput,
    OutputState &finalOutput
);

void getIOStates(
    const RectangleInput &rectangleInput,
    InputState &inputState,
    OutputState &outputState
);

void makeNerfedReport(
    const OutputState &nerfedOutputs,
    gc_report_t *gcReport
);

#endif
