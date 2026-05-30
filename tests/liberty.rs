use vyges_sta_si::liberty::{Dir, Lib};

const LIB: &str = r#"
library (test) {
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

#[test]
fn parses_cell_pins_arcs() {
    let lib = Lib::parse(LIB).unwrap();
    let inv = lib.cell("INV").unwrap();
    assert_eq!(inv.pins["A"].direction, Dir::In);
    assert!((inv.pins["A"].capacitance - 0.002).abs() < 1e-12);
    let y = &inv.pins["Y"];
    assert_eq!(y.direction, Dir::Out);
    assert_eq!(y.arcs.len(), 1);
    assert_eq!(y.arcs[0].related_pin, "A");
    assert_eq!(y.arcs[0].sense, "negative_unate");
    let cr = &y.arcs[0].cell_rise;
    assert_eq!(cr.index_1, vec![0.01, 0.04]);
    assert_eq!(cr.index_2, vec![0.001, 0.004]);
    assert_eq!(cr.values, vec![vec![0.10, 0.20], vec![0.30, 0.40]]);
}

#[test]
fn bilinear_interpolation_and_clamp() {
    let lib = Lib::parse(LIB).unwrap();
    let cr = &lib.cell("INV").unwrap().pins["Y"].arcs[0].cell_rise;
    // exact corners
    assert!((cr.lookup(0.01, 0.001) - 0.10).abs() < 1e-9);
    assert!((cr.lookup(0.04, 0.004) - 0.40).abs() < 1e-9);
    // midpoint in slew, low load -> 0.5*(0.10)+0.5*(0.30) = 0.20
    assert!((cr.lookup(0.025, 0.001) - 0.20).abs() < 1e-9);
    // clamp below grid -> corner value
    assert!((cr.lookup(0.0, 0.0) - 0.10).abs() < 1e-9);
    // clamp above grid -> far corner
    assert!((cr.lookup(9.0, 9.0) - 0.40).abs() < 1e-9);
}
