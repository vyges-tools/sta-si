use vyges_sta_si::liberty::{Dir, Lib};

const LIB: &str = r#"
library (test) {
  lu_table_template (vyges_nldm) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  delay_model : table_lookup;
  cell (INV) {
    pin (A) {
      direction : input;
      capacitance : 0.002;
    }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        timing_sense : negative_unate;
        cell_rise (vyges_nldm) {
          index_1 ("0.01, 0.04");
          index_2 ("0.001, 0.004");
          values ( \
            "0.10, 0.20", \
            "0.30, 0.40" );
        }
        cell_fall (vyges_nldm) {
          index_1 ("0.01, 0.04");
          index_2 ("0.001, 0.004");
          values ( "0.11, 0.21", "0.31, 0.41" );
        }
        rise_transition (vyges_nldm) {
          index_1 ("0.01, 0.04");
          index_2 ("0.001, 0.004");
          values ( "0.05, 0.06", "0.07, 0.08" );
        }
        fall_transition (vyges_nldm) {
          index_1 ("0.01, 0.04");
          index_2 ("0.001, 0.004");
          values ( "0.04, 0.05", "0.06, 0.07" );
        }
      }
    }
  }
}
"#;

/// Liberty numbers are read as the reference's lexer reads them — `strtof`, one rounding to f32 —
/// so a value is the f32 nearest its decimal, and agreement is to float precision (1e-6 here).
fn f32s(v: &[f64]) -> Vec<f64> {
    v.iter().map(|&x| x as f32 as f64).collect()
}

#[test]
fn parses_cell_pins_arcs() {
    let lib = Lib::parse(LIB).unwrap();
    let inv = lib.cell("INV").unwrap();
    assert_eq!(inv.pins["A"].direction, Dir::In);
    assert!((inv.pins["A"].capacitance - 0.002).abs() < 1e-6);
    let y = &inv.pins["Y"];
    assert_eq!(y.direction, Dir::Out);
    assert_eq!(y.arcs.len(), 1);
    assert_eq!(y.arcs[0].related_pin, "A");
    assert_eq!(y.arcs[0].sense, "negative_unate");
    let cr = &y.arcs[0].cell_rise;
    assert_eq!(cr.index_1, f32s(&[0.01, 0.04]));
    assert_eq!(cr.index_2, f32s(&[0.001, 0.004]));
    assert_eq!(cr.values, vec![f32s(&[0.10, 0.20]), f32s(&[0.30, 0.40])]);
}

/// ⛔ EXTRAPOLATES past an axis; it does NOT clamp.
///
/// `findValueIndex` returns index 0 below the axis and `size - 2` above it — its comment
/// reads "Return max_index-1 for value too large so interpolation pts are index,index+1" —
/// and `Table2::findValue` then forms `dx = (x - xl) / (xu - xl)` from the ACTUAL x, which
/// is negative below and greater than 1 above. Clamping makes a cell look load-independent
/// past its characterisation range, which also drives `gateModelRd` to zero and sends
/// `setCeffAlgorithm` down the capacitive branch, degrading no slew at all.
#[test]
fn bilinear_interpolation_extrapolates_past_the_axis() {
    let lib = Lib::parse(LIB).unwrap();
    let cr = &lib.cell("INV").unwrap().pins["Y"].arcs[0].cell_rise;
    // exact corners
    assert!((cr.lookup(0.01, 0.001) - 0.10).abs() < 1e-6);
    assert!((cr.lookup(0.04, 0.004) - 0.40).abs() < 1e-6);
    // midpoint in slew, low load -> 0.5*(0.10)+0.5*(0.30) = 0.20
    assert!((cr.lookup(0.025, 0.001) - 0.20).abs() < 1e-6);
    // one grid step BEYOND the top of the load axis: 0.004 + (0.004-0.001) = 0.007,
    // so the value continues one step past 0.20 -> 0.30 on the low-slew row.
    let step = cr.lookup(0.01, 0.007);
    assert!(
        (step - 0.30).abs() < 1e-6,
        "past the load axis the table must continue linearly, got {step}"
    );
    // and BELOW the axis it continues the other way: 0.001 - 0.003 = -0.002 is one step
    // under, so 0.10 - 0.10 = 0.0.
    let under = cr.lookup(0.01, -0.002);
    assert!(
        (under - 0.0).abs() < 1e-6,
        "below the load axis it must continue linearly too, got {under}"
    );
    // far past both axes it keeps going rather than parking on the corner
    assert!(
        cr.lookup(9.0, 9.0) > 0.40,
        "clamping would return exactly the far corner 0.40"
    );
}

/// A gzipped library (`.lib.gz`, as the reference's asap7 libraries ship) reads as its text and
/// times the same design to the same report. The fixture is `gzip -n -9` of `examples/seq/seq.lib`.
#[test]
fn a_gzipped_library_times_as_its_text() {
    use vyges_sta_si::engine::analyze_job;
    use vyges_sta_si::job::StaJob;
    let plain = Lib::load("examples/seq/seq.lib").expect("seq.lib");
    let gz = Lib::load("examples/seq_gz/seq.lib.gz").expect("seq.lib.gz");
    assert_eq!(format!("{plain:?}"), format!("{gz:?}"), "the library read differs");
    let a = analyze_job(&StaJob::load("examples/seq/seq.sta").unwrap()).unwrap();
    let b = analyze_job(&StaJob::load("examples/seq_gz/seq_gz.sta").unwrap()).unwrap();
    assert_eq!(format!("{a:?}"), format!("{b:?}"), "the report differs");
}
