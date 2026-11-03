/*
  GCode.cpp - rs274/ngc parser.

  Part of Grbl
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  Label cutter changes:
    M3 [S]  head 1 down (S = power, $30 = 100 %)    - synchronized with motion
    M4 [S]  head 2 down                              - synchronized with motion
    M5      both heads up                            - synchronized with motion
    M67 E<0|1> Q<0-100>  fan duty, synchronized with motion
    M68 E<0|1> Q<0-100>  fan duty, immediate
    Any move that turns the feed roller (Z) is rejected with error 150
    when the paper sensor reports an empty tray.
  Removed: probing (G38.x), coolant (M7/M8/M9), digital outputs (M62-M65).
*/

#include "Grbl.h"

static const int32_t MaxLineNumber = 10000000;
static const uint8_t MaxToolNumber = 255;

parser_state_t        gc_state;
static parser_block_t gc_block;

#define FAIL(status) return (status);

enum GCParserFlags : uint8_t {
    GCParserNone           = 0,
    GCParserJogMotion      = bit(0),
    GCParserArcIsClockwise = bit(2),
};

enum class AxisCommand : uint8_t { None = 0, NonModal = 1, MotionMode = 2, ToolLengthOffset = 3 };

void gc_init() {
    memset(&gc_state, 0, sizeof(parser_state_t));
    gc_state.modal.coord_select = CoordIndex::G54;
    gc_state.head_power         = settings.head_s_max;  // Plain M3/M4 = full power
    settings_read_coord(gc_state.modal.coord_select, gc_state.coord_system);
}

void gc_sync_position() {
    system_convert_array_steps_to_mpos(gc_state.position, sys_position);
}

// Remove whitespace and comments, convert to upper case. In place.
static void collapseGCode(char* line) {
    char* parenPtr = NULL;
    char* outPtr   = line;
    char  c;
    for (char* inPtr = line; (c = *inPtr) != '\0'; inPtr++) {
        if (isspace((unsigned char)c)) {
            continue;
        }
        switch (c) {
            case ')':
                parenPtr = NULL;
                break;
            case '(':
                parenPtr = inPtr + 1;
                break;
            case ';':
                *outPtr = '\0';
                return;
            case '%':
            case '\r':
                break;
            default:
                if (!parenPtr) {
                    *outPtr++ = toupper(c);
                }
        }
    }
    *outPtr = '\0';
}

Error gc_execute_line(char* line, uint8_t client) {
    collapseGCode(line);

    memset(&gc_block, 0, sizeof(parser_block_t));
    memcpy(&gc_block.modal, &gc_state.modal, sizeof(gc_modal_t));
    AxisCommand axis_command = AxisCommand::None;
    uint8_t     axis_0, axis_1, axis_linear;
    CoordIndex  coord_select    = CoordIndex::G54;
    uint8_t     axis_words      = 0;
    uint8_t     ijk_words       = 0;
    uint32_t    command_words   = 0;
    uint32_t    value_words     = 0;
    uint8_t     gc_parser_flags = GCParserNone;
    float       coord_data[N_AXIS];
    uint8_t     pValue;

    if (line[0] == '$') {  // "$J=" jog line
        gc_parser_flags |= GCParserJogMotion;
        gc_block.modal.motion    = Motion::Linear;
        gc_block.modal.feed_rate = FeedRate::UnitsPerMin;
    }

    // ------------------------------------------------------------------
    // STEP 2: Import all words
    // ------------------------------------------------------------------
    ModalGroup mg_word_bit;
    uint32_t   bitmask = 0;
    uint8_t    char_counter = (gc_parser_flags & GCParserJogMotion) ? 3 : 0;
    char       letter;
    float      value;
    uint8_t    int_value = 0;
    uint16_t   mantissa  = 0;
    while (line[char_counter] != 0) {
        letter = line[char_counter];
        if ((letter < 'A') || (letter > 'Z')) {
            FAIL(Error::ExpectedCommandLetter);
        }
        char_counter++;
        if (!read_float(line, &char_counter, &value)) {
            FAIL(Error::BadNumberFormat);
        }
        int_value = truncf(value);
        mantissa  = roundf(100 * (value - int_value));
        switch (letter) {
            case 'G':
                switch (int_value) {
                    case 10:
                        gc_block.non_modal_command = NonModal::SetCoordinateData;
                        if (mantissa == 0) {
                            if (axis_command != AxisCommand::None) {
                                FAIL(Error::GcodeAxisCommandConflict);
                            }
                            axis_command = AxisCommand::NonModal;
                        }
                        mg_word_bit = ModalGroup::MG0;
                        break;
                    case 28:
                        gc_block.non_modal_command = mantissa ? NonModal::SetHome0 : NonModal::GoHome0;
                        goto check_mantissa;
                    case 30:
                        gc_block.non_modal_command = mantissa ? NonModal::SetHome1 : NonModal::GoHome1;
                        goto check_mantissa;
                    case 92:
                        gc_block.non_modal_command = mantissa ? NonModal::ResetCoordinateOffset : NonModal::SetCoordinateOffset;
                    check_mantissa:
                        switch (mantissa) {
                            case 0:
                                if (axis_command != AxisCommand::None) {
                                    FAIL(Error::GcodeAxisCommandConflict);
                                }
                                axis_command = AxisCommand::NonModal;
                                break;
                            case 10:
                                mantissa = 0;
                                break;
                            default:
                                FAIL(Error::GcodeUnsupportedCommand);
                        }
                        mg_word_bit = ModalGroup::MG0;
                        break;
                    case 4:
                        gc_block.non_modal_command = NonModal::Dwell;
                        mg_word_bit                = ModalGroup::MG0;
                        break;
                    case 53:
                        gc_block.non_modal_command = NonModal::AbsoluteOverride;
                        mg_word_bit                = ModalGroup::MG0;
                        break;
                    case 0:
                        axis_command          = AxisCommand::MotionMode;
                        gc_block.modal.motion = Motion::Seek;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 1:
                        axis_command          = AxisCommand::MotionMode;
                        gc_block.modal.motion = Motion::Linear;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 2:
                        axis_command          = AxisCommand::MotionMode;
                        gc_block.modal.motion = Motion::CwArc;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 3:
                        axis_command          = AxisCommand::MotionMode;
                        gc_block.modal.motion = Motion::CcwArc;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 80:
                        gc_block.modal.motion = Motion::None;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 17:
                        gc_block.modal.plane_select = Plane::XY;
                        mg_word_bit                 = ModalGroup::MG2;
                        break;
                    case 18:
                        gc_block.modal.plane_select = Plane::ZX;
                        mg_word_bit                 = ModalGroup::MG2;
                        break;
                    case 19:
                        gc_block.modal.plane_select = Plane::YZ;
                        mg_word_bit                 = ModalGroup::MG2;
                        break;
                    case 90:
                        if (mantissa != 0) {
                            FAIL(Error::GcodeUnsupportedCommand);  // G90.1 not supported
                        }
                        gc_block.modal.distance = Distance::Absolute;
                        mg_word_bit             = ModalGroup::MG3;
                        break;
                    case 91:
                        if (mantissa == 0) {
                            gc_block.modal.distance = Distance::Incremental;
                            mg_word_bit             = ModalGroup::MG3;
                        } else if (mantissa == 10) {
                            mantissa    = 0;  // G91.1 is the only arc mode
                            mg_word_bit = ModalGroup::MG4;
                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        break;
                    case 93:
                        gc_block.modal.feed_rate = FeedRate::InverseTime;
                        mg_word_bit              = ModalGroup::MG5;
                        break;
                    case 94:
                        gc_block.modal.feed_rate = FeedRate::UnitsPerMin;
                        mg_word_bit              = ModalGroup::MG5;
                        break;
                    case 20:
                        gc_block.modal.units = Units::Inches;
                        mg_word_bit          = ModalGroup::MG6;
                        break;
                    case 21:
                        gc_block.modal.units = Units::Mm;
                        mg_word_bit          = ModalGroup::MG6;
                        break;
                    case 40:
                        mg_word_bit = ModalGroup::MG7;
                        break;
                    case 43:
                    case 49:
                        if (axis_command != AxisCommand::None) {
                            FAIL(Error::GcodeAxisCommandConflict);
                        }
                        axis_command = AxisCommand::ToolLengthOffset;
                        if (int_value == 49) {
                            gc_block.modal.tool_length = ToolLengthOffset::Cancel;
                        } else if (mantissa == 10) {
                            gc_block.modal.tool_length = ToolLengthOffset::EnableDynamic;
                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        mantissa    = 0;
                        mg_word_bit = ModalGroup::MG8;
                        break;
                    case 54:
                    case 55:
                    case 56:
                    case 57:
                    case 58:
                    case 59:
                        gc_block.modal.coord_select = static_cast<CoordIndex>(int_value - 54);
                        mg_word_bit                 = ModalGroup::MG12;
                        break;
                    case 61:
                        if (mantissa != 0) {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        mg_word_bit = ModalGroup::MG13;
                        break;
                    default:
                        FAIL(Error::GcodeUnsupportedCommand);
                }
                if (mantissa > 0) {
                    FAIL(Error::GcodeCommandValueNotInteger);
                }
                bitmask = bit(mg_word_bit);
                if (bit_istrue(command_words, bitmask)) {
                    FAIL(Error::GcodeModalGroupViolation);
                }
                command_words |= bitmask;
                break;

            case 'M':
                if (mantissa > 0) {
                    FAIL(Error::GcodeCommandValueNotInteger);
                }
                switch (int_value) {
                    case 0:
                        gc_block.modal.program_flow = ProgramFlow::Paused;
                        mg_word_bit                 = ModalGroup::MM4;
                        break;
                    case 1:
                        mg_word_bit = ModalGroup::MM4;
                        break;
                    case 2:
                        gc_block.modal.program_flow = ProgramFlow::CompletedM2;
                        mg_word_bit                 = ModalGroup::MM4;
                        break;
                    case 30:
                        gc_block.modal.program_flow = ProgramFlow::CompletedM30;
                        mg_word_bit                 = ModalGroup::MM4;
                        break;
                    case 3:
                        gc_block.modal.head = HeadState::Head1;
                        mg_word_bit         = ModalGroup::MM7;
                        break;
                    case 4:
                        gc_block.modal.head = HeadState::Head2;
                        mg_word_bit         = ModalGroup::MM7;
                        break;
                    case 5:
                        gc_block.modal.head = HeadState::Off;
                        mg_word_bit         = ModalGroup::MM7;
                        break;
                    case 6:
                        mg_word_bit = ModalGroup::MM6;  // Accepted and ignored
                        break;
                    case 67:
                        gc_block.modal.io_control = IoControl::SetAnalogSync;
                        mg_word_bit               = ModalGroup::MM10;
                        break;
                    case 68:
                        gc_block.modal.io_control = IoControl::SetAnalogImmediate;
                        mg_word_bit               = ModalGroup::MM10;
                        break;
                    default:
                        FAIL(Error::GcodeUnsupportedCommand);
                }
                bitmask = bit(mg_word_bit);
                if (bit_istrue(command_words, bitmask)) {
                    FAIL(Error::GcodeModalGroupViolation);
                }
                command_words |= bitmask;
                break;

            default: {
                GCodeWord axis_word_bit;
                switch (letter) {
                    case 'E':
                        axis_word_bit     = GCodeWord::E;
                        gc_block.values.e = int_value;
                        break;
                    case 'F':
                        axis_word_bit     = GCodeWord::F;
                        gc_block.values.f = value;
                        break;
                    case 'I':
                        axis_word_bit               = GCodeWord::I;
                        gc_block.values.ijk[X_AXIS] = value;
                        ijk_words |= bit(X_AXIS);
                        break;
                    case 'J':
                        axis_word_bit               = GCodeWord::J;
                        gc_block.values.ijk[Y_AXIS] = value;
                        ijk_words |= bit(Y_AXIS);
                        break;
                    case 'K':
                        axis_word_bit               = GCodeWord::K;
                        gc_block.values.ijk[Z_AXIS] = value;
                        ijk_words |= bit(Z_AXIS);
                        break;
                    case 'L':
                        axis_word_bit     = GCodeWord::L;
                        gc_block.values.l = int_value;
                        break;
                    case 'N':
                        axis_word_bit     = GCodeWord::N;
                        gc_block.values.n = truncf(value);
                        break;
                    case 'P':
                        axis_word_bit     = GCodeWord::P;
                        gc_block.values.p = value;
                        break;
                    case 'Q':
                        axis_word_bit     = GCodeWord::Q;
                        gc_block.values.q = value;
                        break;
                    case 'R':
                        axis_word_bit     = GCodeWord::R;
                        gc_block.values.r = value;
                        break;
                    case 'S':
                        axis_word_bit     = GCodeWord::S;
                        gc_block.values.s = value;
                        break;
                    case 'T':
                        axis_word_bit = GCodeWord::T;
                        if (value > MaxToolNumber) {
                            FAIL(Error::GcodeMaxValueExceeded);
                        }
                        gc_state.tool = int_value;
                        break;
                    case 'X':
                        axis_word_bit               = GCodeWord::X;
                        gc_block.values.xyz[X_AXIS] = value;
                        axis_words |= bit(X_AXIS);
                        break;
                    case 'Y':
                        axis_word_bit               = GCodeWord::Y;
                        gc_block.values.xyz[Y_AXIS] = value;
                        axis_words |= bit(Y_AXIS);
                        break;
                    case 'Z':
                        axis_word_bit               = GCodeWord::Z;
                        gc_block.values.xyz[Z_AXIS] = value;
                        axis_words |= bit(Z_AXIS);
                        break;
                    default:
                        FAIL(Error::GcodeUnsupportedCommand);
                }
                bitmask = bit(axis_word_bit);
                if (bit_istrue(value_words, bitmask)) {
                    FAIL(Error::GcodeWordRepeated);
                }
                if (bitmask & (bit(GCodeWord::F) | bit(GCodeWord::N) | bit(GCodeWord::P) | bit(GCodeWord::T) | bit(GCodeWord::S))) {
                    if (value < 0.0f) {
                        FAIL(Error::NegativeValue);
                    }
                }
                value_words |= bitmask;
            }
        }
    }

    // ------------------------------------------------------------------
    // STEP 3: Error-check all commands and values
    // ------------------------------------------------------------------
    if (axis_words && axis_command == AxisCommand::None) {
        axis_command = AxisCommand::MotionMode;
    }
    if (bit_istrue(value_words, bit(GCodeWord::N)) && gc_block.values.n > MaxLineNumber) {
        FAIL(Error::GcodeInvalidLineNumber);
    }
    // Feed rate
    if (gc_parser_flags & GCParserJogMotion) {
        if (bit_isfalse(value_words, bit(GCodeWord::F))) {
            FAIL(Error::GcodeUndefinedFeedRate);
        }
        if (gc_block.modal.units == Units::Inches) {
            gc_block.values.f *= MM_PER_INCH;
        }
    } else {
        if (gc_block.modal.feed_rate == FeedRate::InverseTime) {
            if (axis_command == AxisCommand::MotionMode) {
                if ((gc_block.modal.motion != Motion::None) && (gc_block.modal.motion != Motion::Seek)) {
                    if (bit_isfalse(value_words, bit(GCodeWord::F))) {
                        FAIL(Error::GcodeUndefinedFeedRate);
                    }
                }
            }
        } else {
            if (gc_state.modal.feed_rate == FeedRate::UnitsPerMin) {
                if (bit_istrue(value_words, bit(GCodeWord::F))) {
                    if (gc_block.modal.units == Units::Inches) {
                        gc_block.values.f *= MM_PER_INCH;
                    }
                } else {
                    gc_block.values.f = gc_state.feed_rate;
                }
            }
        }
    }
    // Head power (S)
    if (bit_isfalse(value_words, bit(GCodeWord::S))) {
        gc_block.values.s = gc_state.head_power;
    }
    // Dwell
    if (gc_block.non_modal_command == NonModal::Dwell) {
        if (bit_isfalse(value_words, bit(GCodeWord::P))) {
            FAIL(Error::GcodeValueWordMissing);
        }
        bit_false(value_words, bit(GCodeWord::P));
    }
    // Fans
    if (gc_block.modal.io_control != IoControl::None) {
        if (bit_isfalse(value_words, bit(GCodeWord::E)) || bit_isfalse(value_words, bit(GCodeWord::Q))) {
            FAIL(Error::GcodeValueWordMissing);
        }
        if (gc_block.values.e > 1) {
            FAIL(Error::PParamMaxExceeded);
        }
        bit_false(value_words, bit(GCodeWord::E) | bit(GCodeWord::Q));
    }
    // Plane
    switch (gc_block.modal.plane_select) {
        case Plane::XY:
            axis_0      = X_AXIS;
            axis_1      = Y_AXIS;
            axis_linear = Z_AXIS;
            break;
        case Plane::ZX:
            axis_0      = Z_AXIS;
            axis_1      = X_AXIS;
            axis_linear = Y_AXIS;
            break;
        default:
            axis_0      = Y_AXIS;
            axis_1      = Z_AXIS;
            axis_linear = X_AXIS;
    }
    // Units
    if (gc_block.modal.units == Units::Inches) {
        for (int idx = 0; idx < N_AXIS; idx++) {
            if (bit_istrue(axis_words, bit(idx))) {
                gc_block.values.xyz[idx] *= MM_PER_INCH;
            }
        }
    }
    // Tool length offset (applied to Z)
    if (axis_command == AxisCommand::ToolLengthOffset) {
        if (gc_block.modal.tool_length == ToolLengthOffset::EnableDynamic) {
            if (axis_words ^ bit(Z_AXIS)) {
                FAIL(Error::GcodeG43DynamicAxisError);
            }
        }
    }
    // Coordinate system selection
    float block_coord_system[N_AXIS];
    memcpy(block_coord_system, gc_state.coord_system, sizeof(gc_state.coord_system));
    if (bit_istrue(command_words, bit(ModalGroup::MG12))) {
        if (gc_block.modal.coord_select >= CoordIndex::NWCSystems) {
            FAIL(Error::GcodeUnsupportedCoordSys);
        }
        if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
            settings_read_coord(gc_block.modal.coord_select, block_coord_system);
        }
    }
    // Non-modal actions
    switch (gc_block.non_modal_command) {
        case NonModal::SetCoordinateData:
            if (!axis_words) {
                FAIL(Error::GcodeNoAxisWords);
            }
            if (bit_isfalse(value_words, (bit(GCodeWord::P) | bit(GCodeWord::L)))) {
                FAIL(Error::GcodeValueWordMissing);
            }
            if (gc_block.values.l != 20) {
                if (gc_block.values.l == 2) {
                    if (bit_istrue(value_words, bit(GCodeWord::R))) {
                        FAIL(Error::GcodeUnsupportedCommand);
                    }
                } else {
                    FAIL(Error::GcodeUnsupportedCommand);
                }
            }
            pValue       = truncf(gc_block.values.p);
            coord_select = pValue > 0 ? static_cast<CoordIndex>(pValue - 1) : gc_block.modal.coord_select;
            if (coord_select >= CoordIndex::NWCSystems) {
                FAIL(Error::GcodeUnsupportedCoordSys);
            }
            bit_false(value_words, (bit(GCodeWord::L) | bit(GCodeWord::P)));
            settings_read_coord(coord_select, coord_data);
            for (int idx = 0; idx < N_AXIS; idx++) {
                if (bit_istrue(axis_words, bit(idx))) {
                    if (gc_block.values.l == 20) {
                        coord_data[idx] = gc_state.position[idx] - gc_state.coord_offset[idx] - gc_block.values.xyz[idx];
                        if (idx == Z_AXIS) {
                            coord_data[idx] -= gc_state.tool_length_offset;
                        }
                    } else {
                        coord_data[idx] = gc_block.values.xyz[idx];
                    }
                }
            }
            break;
        case NonModal::SetCoordinateOffset:
            if (!axis_words) {
                FAIL(Error::GcodeNoAxisWords);
            }
            for (int idx = 0; idx < N_AXIS; idx++) {
                if (bit_istrue(axis_words, bit(idx))) {
                    gc_block.values.xyz[idx] = gc_state.position[idx] - block_coord_system[idx] - gc_block.values.xyz[idx];
                    if (idx == Z_AXIS) {
                        gc_block.values.xyz[idx] -= gc_state.tool_length_offset;
                    }
                } else {
                    gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
                }
            }
            break;
        default:
            if (axis_command != AxisCommand::ToolLengthOffset) {
                if (axis_words) {
                    for (int idx = 0; idx < N_AXIS; idx++) {
                        if (bit_isfalse(axis_words, bit(idx))) {
                            gc_block.values.xyz[idx] = gc_state.position[idx];
                        } else if (gc_block.non_modal_command != NonModal::AbsoluteOverride) {
                            if (gc_block.modal.distance == Distance::Absolute) {
                                gc_block.values.xyz[idx] += block_coord_system[idx] + gc_state.coord_offset[idx];
                                if (idx == Z_AXIS) {
                                    gc_block.values.xyz[idx] += gc_state.tool_length_offset;
                                }
                            } else {
                                gc_block.values.xyz[idx] += gc_state.position[idx];
                            }
                        }
                    }
                }
            }
            switch (gc_block.non_modal_command) {
                case NonModal::GoHome0:
                case NonModal::GoHome1:
                    settings_read_coord(gc_block.non_modal_command == NonModal::GoHome0 ? CoordIndex::G28 : CoordIndex::G30, coord_data);
                    if (axis_words) {
                        for (int idx = 0; idx < N_AXIS; idx++) {
                            if (!(axis_words & bit(idx))) {
                                coord_data[idx] = gc_state.position[idx];
                            }
                        }
                    } else {
                        axis_command = AxisCommand::None;
                    }
                    break;
                case NonModal::AbsoluteOverride:
                    if (!(gc_block.modal.motion == Motion::Seek || gc_block.modal.motion == Motion::Linear)) {
                        FAIL(Error::GcodeG53InvalidMotionMode);
                    }
                    break;
                default:
                    break;
            }
    }
    // Motion modes
    if (gc_block.modal.motion == Motion::None) {
        if (axis_words) {
            FAIL(Error::GcodeAxisWordsExist);
        }
    } else if (axis_command == AxisCommand::MotionMode) {
        if (gc_block.modal.motion == Motion::Seek) {
            if (!axis_words) {
                axis_command = AxisCommand::None;
            }
        } else {
            if (gc_block.values.f == 0.0f) {
                FAIL(Error::GcodeUndefinedFeedRate);
            }
            switch (gc_block.modal.motion) {
                case Motion::Linear:
                    if (!axis_words) {
                        axis_command = AxisCommand::None;
                    }
                    break;
                case Motion::CwArc:
                    gc_parser_flags |= GCParserArcIsClockwise;
                    // fall through
                case Motion::CcwArc: {
                    if (!axis_words) {
                        FAIL(Error::GcodeNoAxisWords);
                    }
                    if (!(axis_words & (bit(axis_0) | bit(axis_1)))) {
                        FAIL(Error::GcodeNoAxisWordsInPlane);
                    }
                    float x = gc_block.values.xyz[axis_0] - gc_state.position[axis_0];
                    float y = gc_block.values.xyz[axis_1] - gc_state.position[axis_1];
                    if (value_words & bit(GCodeWord::R)) {
                        bit_false(value_words, bit(GCodeWord::R));
                        if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) {
                            FAIL(Error::GcodeInvalidTarget);
                        }
                        if (gc_block.modal.units == Units::Inches) {
                            gc_block.values.r *= MM_PER_INCH;
                        }
                        float h_x2_div_d = 4.0f * gc_block.values.r * gc_block.values.r - x * x - y * y;
                        if (h_x2_div_d < 0) {
                            FAIL(Error::GcodeArcRadiusError);
                        }
                        h_x2_div_d = -sqrtf(h_x2_div_d) / sqrtf(x * x + y * y);
                        if (gc_block.modal.motion == Motion::CcwArc) {
                            h_x2_div_d = -h_x2_div_d;
                        }
                        if (gc_block.values.r < 0) {
                            h_x2_div_d        = -h_x2_div_d;
                            gc_block.values.r = -gc_block.values.r;
                        }
                        gc_block.values.ijk[axis_0] = 0.5f * (x - (y * h_x2_div_d));
                        gc_block.values.ijk[axis_1] = 0.5f * (y + (x * h_x2_div_d));
                    } else {
                        if (!(ijk_words & (bit(axis_0) | bit(axis_1)))) {
                            FAIL(Error::GcodeNoOffsetsInPlane);
                        }
                        bit_false(value_words, (bit(GCodeWord::I) | bit(GCodeWord::J) | bit(GCodeWord::K)));
                        if (gc_block.modal.units == Units::Inches) {
                            for (int idx = 0; idx < 3; idx++) {
                                if (ijk_words & bit(idx)) {
                                    gc_block.values.ijk[idx] *= MM_PER_INCH;
                                }
                            }
                        }
                        x -= gc_block.values.ijk[axis_0];
                        y -= gc_block.values.ijk[axis_1];
                        float target_r    = sqrtf(x * x + y * y);
                        gc_block.values.r = sqrtf(gc_block.values.ijk[axis_0] * gc_block.values.ijk[axis_0] +
                                                  gc_block.values.ijk[axis_1] * gc_block.values.ijk[axis_1]);
                        float delta_r     = fabsf(target_r - gc_block.values.r);
                        if (delta_r > 0.005f) {
                            if (delta_r > 0.5f) {
                                FAIL(Error::GcodeInvalidTarget);
                            }
                            if (delta_r > (0.001f * gc_block.values.r)) {
                                FAIL(Error::GcodeInvalidTarget);
                            }
                        }
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
    // Feed-motor interlock: refuse any block that would turn the feed roller without paper.
    if (sys.state != State::CheckMode) {
        bool z_moves = false;
        if (axis_command == AxisCommand::MotionMode && gc_block.modal.motion != Motion::None) {
            z_moves = gc_block.values.xyz[Z_AXIS] != gc_state.position[Z_AXIS];
        }
        if (gc_block.non_modal_command == NonModal::GoHome0 || gc_block.non_modal_command == NonModal::GoHome1) {
            z_moves = z_moves || coord_data[Z_AXIS] != gc_state.position[Z_AXIS];
        }
        if (z_moves && !machine_paper_present()) {
            FAIL(Error::NoPaper);
        }
    }
    // Unused words
    if (gc_parser_flags & GCParserJogMotion) {
        bit_false(value_words, (bit(GCodeWord::N) | bit(GCodeWord::F)));
    } else {
        bit_false(value_words, (bit(GCodeWord::N) | bit(GCodeWord::F) | bit(GCodeWord::S) | bit(GCodeWord::T)));
    }
    if (axis_command != AxisCommand::None) {
        bit_false(value_words, (bit(GCodeWord::X) | bit(GCodeWord::Y) | bit(GCodeWord::Z)));
    }
    if (value_words) {
        FAIL(Error::GcodeUnusedWords);
    }

    // ------------------------------------------------------------------
    // STEP 4: Execute
    // ------------------------------------------------------------------
    plan_line_data_t plan_data;
    plan_line_data_t* pl_data = &plan_data;
    memset(pl_data, 0, sizeof(plan_line_data_t));

    if (gc_parser_flags & GCParserJogMotion) {
        if (command_words & ~(bit(ModalGroup::MG3) | bit(ModalGroup::MG6) | bit(ModalGroup::MG0))) {
            FAIL(Error::InvalidJogCommand);
        }
        if (!(gc_block.non_modal_command == NonModal::AbsoluteOverride || gc_block.non_modal_command == NonModal::NoAction)) {
            FAIL(Error::InvalidJogCommand);
        }
        bool  cancelledInflight = false;
        Error status            = jog_execute(pl_data, &gc_block, &cancelledInflight);
        if (status == Error::Ok && !cancelledInflight) {
            memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));
        }
        return status == Error::JogCancelled ? Error::Ok : status;
    }

    bool outputs_live = sys.state != State::CheckMode;

    gc_state.line_number = gc_block.values.n;
    pl_data->line_number = gc_state.line_number;
    gc_state.modal.feed_rate = gc_block.modal.feed_rate;
    if (gc_state.modal.feed_rate == FeedRate::InverseTime) {
        pl_data->motion.inverseTime = 1;
    }
    gc_state.feed_rate = gc_block.values.f;
    pl_data->feed_rate = gc_state.feed_rate;

    // Head power / selection (synchronized with the motion queue)
    bool head_changed  = gc_state.modal.head != gc_block.modal.head;
    bool power_changed = gc_state.head_power != gc_block.values.s;
    gc_state.head_power = gc_block.values.s;
    if (head_changed || (power_changed && gc_block.modal.head != HeadState::Off)) {
        gc_state.modal.head = gc_block.modal.head;
        if (outputs_live) {
            heads_sync(gc_state.modal.head, gc_state.head_power);
        }
    }

    // Fans
    if (gc_block.modal.io_control != IoControl::None && outputs_live) {
        if (gc_block.modal.io_control == IoControl::SetAnalogSync) {
            protocol_buffer_synchronize();
        }
        fans_set(gc_block.values.e, constrain(gc_block.values.q, 0.0f, 100.0f));
    }

    if (gc_block.non_modal_command == NonModal::Dwell) {
        mc_dwell(int32_t(gc_block.values.p * 1000.0f));
    }
    gc_state.modal.plane_select = gc_block.modal.plane_select;
    gc_state.modal.units        = gc_block.modal.units;

    if (axis_command == AxisCommand::ToolLengthOffset) {
        gc_state.modal.tool_length = gc_block.modal.tool_length;
        if (gc_state.modal.tool_length == ToolLengthOffset::Cancel) {
            gc_block.values.xyz[Z_AXIS] = 0.0f;
        }
        if (gc_state.tool_length_offset != gc_block.values.xyz[Z_AXIS]) {
            gc_state.tool_length_offset = gc_block.values.xyz[Z_AXIS];
            system_flag_wco_change();
        }
    }
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
        gc_state.modal.coord_select = gc_block.modal.coord_select;
        memcpy(gc_state.coord_system, block_coord_system, sizeof(gc_state.coord_system));
        system_flag_wco_change();
    }
    gc_state.modal.distance = gc_block.modal.distance;

    switch (gc_block.non_modal_command) {
        case NonModal::SetCoordinateData:
            settings_store_coord(coord_select, coord_data);
            if (gc_state.modal.coord_select == coord_select) {
                memcpy(gc_state.coord_system, coord_data, sizeof(gc_state.coord_system));
                system_flag_wco_change();
            }
            break;
        case NonModal::GoHome0:
        case NonModal::GoHome1:
            pl_data->motion.rapidMotion = 1;
            if (axis_command != AxisCommand::None) {
                limits_check_soft(gc_block.values.xyz);
                mc_line(gc_block.values.xyz, pl_data);
            }
            limits_check_soft(coord_data);
            mc_line(coord_data, pl_data);
            memcpy(gc_state.position, coord_data, sizeof(gc_state.position));
            break;
        case NonModal::SetHome0:
            settings_store_coord(CoordIndex::G28, gc_state.position);
            break;
        case NonModal::SetHome1:
            settings_store_coord(CoordIndex::G30, gc_state.position);
            break;
        case NonModal::SetCoordinateOffset:
            memcpy(gc_state.coord_offset, gc_block.values.xyz, sizeof(gc_block.values.xyz));
            system_flag_wco_change();
            break;
        case NonModal::ResetCoordinateOffset:
            clear_vector(gc_state.coord_offset);
            system_flag_wco_change();
            break;
        default:
            break;
    }

    gc_state.modal.motion = gc_block.modal.motion;
    if (gc_state.modal.motion != Motion::None && axis_command == AxisCommand::MotionMode) {
        if (gc_state.modal.motion == Motion::Linear) {
            limits_check_soft(gc_block.values.xyz);
            mc_line(gc_block.values.xyz, pl_data);
        } else if (gc_state.modal.motion == Motion::Seek) {
            pl_data->motion.rapidMotion = 1;
            limits_check_soft(gc_block.values.xyz);
            mc_line(gc_block.values.xyz, pl_data);
        } else {
            mc_arc(gc_block.values.xyz,
                   pl_data,
                   gc_state.position,
                   gc_block.values.ijk,
                   gc_block.values.r,
                   axis_0,
                   axis_1,
                   axis_linear,
                   bit_istrue(gc_parser_flags, GCParserArcIsClockwise));
        }
        memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));
    }

    // Program flow
    gc_state.modal.program_flow = gc_block.modal.program_flow;
    switch (gc_state.modal.program_flow) {
        case ProgramFlow::Paused:
            protocol_buffer_synchronize();
            if (sys.state != State::CheckMode) {
                rt_set(ExecFlag::FeedHold);
                protocol_execute_realtime();
            }
            break;
        case ProgramFlow::CompletedM2:
        case ProgramFlow::CompletedM30:
            protocol_buffer_synchronize();
            gc_state.modal.motion       = Motion::Linear;
            gc_state.modal.plane_select = Plane::XY;
            gc_state.modal.distance     = Distance::Absolute;
            gc_state.modal.feed_rate    = FeedRate::UnitsPerMin;
            gc_state.modal.coord_select = CoordIndex::G54;
            gc_state.modal.head         = HeadState::Off;
            sys.f_override              = FEED_OVERRIDE_DEFAULT;
            sys.r_override              = RAPID_OVERRIDE_DEFAULT;
            sys_rt_f_override           = FEED_OVERRIDE_DEFAULT;
            sys_rt_r_override           = RAPID_OVERRIDE_DEFAULT;
            if (sys.state != State::CheckMode) {
                settings_read_coord(gc_state.modal.coord_select, gc_state.coord_system);
                system_flag_wco_change();
                heads_set(HeadState::Off, 0);
            }
            report_feedback_message(Message::ProgramEnd);
            break;
        default:
            break;
    }
    gc_state.modal.program_flow = ProgramFlow::Running;
    return Error::Ok;
}
