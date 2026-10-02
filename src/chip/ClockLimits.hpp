#pragma once
// The SAM D21's clock limits, for chip_atsam_common/ClockSolver.hpp. Each number from DS40001882L
// ("line" is the line of SAMD21_DA1_Family_Datasheet.md) and its errata DS80000760M
// (SAMD21_DA1_Errata.md).
#include "atsam_common/ClockSolver.hpp"

namespace Kvasir::ClockLimits::D21 {
// FDPLL96M: f_IN 32..2000 kHz, f_OUT 48..96 MHz (Tables 37-58/59/60, lines 42239/42257/42275,
// identical for every variant and revision); f_GCLK_DPLL max 2 MHz (Table 37-7, line 40598). LDR 12
// bits, LDRFRAC 4 bits (17.8.18, lines 9434-9436); DIV 11 bits (17.8.19, line 9472); f_ref = f_xosc
// / (2 (DIV + 1)) and f_out = f_ref (LDR + 1 + LDRFRAC / 16) (17.6.8.3, line 9409 ff., both
// formulas read on the rendered page 173: the text loses the parentheses). No output prescaler.
inline constexpr Dpll                   Fdpll96m{.refMin   = 32'000,
                                                 .refMax   = 2'000'000,
                                                 .outMin   = 48'000'000,
                                                 .outMax   = 96'000'000,
                                                 .ldrBits  = 12,
                                                 .fracBits = 4,
                                                 .divBits  = 11,
                                                 .prescMax = 0};
inline constexpr Prescaler::FixedString DpllWhere = "SAM D21 FDPLL96M, Tables 37-58/59/60";

// f_CPU and f_AHB max 48 MHz (Table 37-7, line 40598)
inline constexpr std::uint64_t CpuMax = 48'000'000;

// The FDPLL96M "operation above 64 MHz is not functional below 0 C" (errata 1.3.2, line 426, no
// workaround): revisions A-D (errata PDF page 9; the converted text's table reads like A-F). The
// solver's "lowest DCO first" keeps away from it.
inline constexpr std::uint64_t DcoMaxBelow0C = 64'000'000;

// Which wait-state table holds for the part (the ordering code's temperature grade and, for the
// AEC-Q100 parts, the device variant). The 105 C chapter (38) has no wait-state table and does not
// refer to another one: no guess, refused.
enum class Rating : std::uint8_t {
    c85,                // Table 37-42 (85 C)
    c105,               // chapter 38: no table
    c125,               // Table 39-28
    aecQ100VariantA,    // Table 40-37
    aecQ100VariantBD,   // Table 40-38
};

// Table 37-42 "Maximum Operating Frequency" (85 C), line 41910
inline constexpr WaitStateTable WaitStates85C{
  "SAM D21 Table 37-42",
  {{
    {Supply::from1V62, {14'000'000, 28'000'000, 42'000'000, 48'000'000}},
    {Supply::from2V7, {24'000'000, 48'000'000}},
  }}};
// Table 39-28 (125 C), line 44739
inline constexpr WaitStateTable WaitStates125C{
  "SAM D21 Table 39-28 (125 C)",
  {{
    {Supply::from1V62, {14'000'000, 28'000'000, 40'000'000}},
    {Supply::from2V7, {24'000'000, 40'000'000}},
  }}};
// Table 40-37 (AEC-Q100, device variant A), line 46433: 2.7-3.63 V only
inline constexpr WaitStateTable WaitStatesAecQ100A{"SAM D21 Table 40-37 (AEC-Q100, variant A)",
                                                   {{
                                                     {Supply::from2V7, {24'000'000, 40'000'000}},
                                                     {},
                                                   }}};
// Table 40-38 (AEC-Q100, device variants B and D), line 46442: 2.7-3.63 V only
inline constexpr WaitStateTable WaitStatesAecQ100BD{"SAM D21 Table 40-38 (AEC-Q100, variants B, D)",
                                                    {{
                                                      {Supply::from2V7, {24'000'000, 48'000'000}},
                                                      {},
                                                    }}};

consteval WaitStateTable const* waitStateTable(Rating r) {
    switch(r) {
    case Rating::c85:              return &WaitStates85C;
    case Rating::c105:             return nullptr;
    case Rating::c125:             return &WaitStates125C;
    case Rating::aecQ100VariantA:  return &WaitStatesAecQ100A;
    case Rating::aecQ100VariantBD: return &WaitStatesAecQ100BD;
    }
    return nullptr;
}

// NVMCTRL.CTRLB.RWS is 4 bits (22.8.2, line 16110); it has to match before the AHB clock changes
// (22.5.2, line 15653).
inline constexpr unsigned RwsMax = 15;

// The fewest wait states for Hz at this supply and rating; a compile error naming the table when it
// has none.
template<std::uint64_t Hz,
         Supply        S,
         Rating        R = Rating::c85>
consteval unsigned waitStates() {
    static_assert(R != Rating::c105,
                  "SAM D21 at 105 C: DS40001882L chapter 38 has no wait-state table and names "
                  "none to use; write RWS by hand with the reason");
    if constexpr(R == Rating::c85) {
        return Nvm::waitStatesFrom<Hz, S, WaitStates85C>();
    } else if constexpr(R == Rating::c125) {
        return Nvm::waitStatesFrom<Hz, S, WaitStates125C>();
    } else if constexpr(R == Rating::aecQ100VariantA) {
        return Nvm::waitStatesFrom<Hz, S, WaitStatesAecQ100A>();
    } else {
        return Nvm::waitStatesFrom<Hz, S, WaitStatesAecQ100BD>();
    }
}

// DFLL48M closed loop: f_REF 0.732..33 kHz (Table 37-54, line 42158; Table 37-53 the same for
// variant A; f_GCLK_DFLL48M_REF max 33 kHz in Table 37-7); DFLLMUL.MUL 16 bits (17.8.12, line
// 9132); DFLLVAL.COARSE 6 bits, FINE 10 bits (17.8.11, lines 9086/9090).
inline constexpr Dfll Dfll48m{.refMin     = 732,
                              .refMax     = 33'000,
                              .mulBits    = 16,
                              .coarseBits = 6,
                              .fineBits   = 10};

// "any reference clock for DFLL in close loop must be within a 2% error accuracy" (note 1 under
// Table 37-54): what the caller states about its reference, at most this
inline constexpr std::uint64_t DfllReferenceAccuracyMaxPpm = 20'000;

// Open loop: 45 / 47 / 49 MHz min / typ / max with COARSE CAL and FINE 512 (Table 37-52, line
// 42134). 49 MHz is above f_CPU max (48 MHz, Table 37-7) and above the last row of Table 37-42: the
// datasheet gives no wait-state count for the open loop's worst case.
inline constexpr std::uint64_t DfllOpenLoopMaxHz = 49'000'000;

// GCLK generators: DIV bits and "Maximum Division Factor" (15.8.5 GENDIV, line 5407 ff.): generator
// 0 8 bits / 512, 1 16 bits / 131072, 2 5 bits / 64, 3-8 8 bits / 512. DIVSEL = 1 divides by
// 2^(DIV + 1) (15.8.4, line 5280).
consteval unsigned gclkDivBits(unsigned generator) {
    return generator == 1 ? 16 : (generator == 2 ? 5 : 8);
}

consteval unsigned long long gclkMaxDivision(unsigned generator) {
    return 1ULL << (gclkDivBits(generator) + 1);
}
}   // namespace Kvasir::ClockLimits::D21
