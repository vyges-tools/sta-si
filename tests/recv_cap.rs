// CCS receiver capacitance: sta-si parses the receiver_capacitance group emitted
// by vyges-char and uses the Miller-aware load (C1/C2) instead of the static
// `capacitance` as the driver's pin load.
use vyges_sta_si::liberty::Lib;

const LIB: &str = r#"
library (test) {
  delay_model : table_lookup;
  cell (INV) {
    pin (A) {
      direction : input;
      capacitance : 0.0019;
      receiver_capacitance () {
        receiver_capacitance1_rise (t) {
          index_1 ("0.05"); index_2 ("0.005");
          values ( "0.0020" );
        }
        receiver_capacitance2_rise (t) {
          index_1 ("0.05"); index_2 ("0.005");
          values ( "0.0030" );
        }
        receiver_capacitance1_fall (t) {
          index_1 ("0.05"); index_2 ("0.005");
          values ( "0.0018" );
        }
        receiver_capacitance2_fall (t) {
          index_1 ("0.05"); index_2 ("0.005");
          values ( "0.0040" );
        }
      }
    }
    pin (Y) { direction : output; }
  }
  cell (BUF) {
    pin (A) { direction : input; capacitance : 0.0019; }
    pin (Y) { direction : output; }
  }
}
"#;

#[test]
fn parses_receiver_capacitance() {
    let lib = Lib::parse(LIB).unwrap();
    let a = &lib.cell("INV").unwrap().pins["A"];
    let r = a.recv.as_ref().expect("receiver_capacitance parsed");
    assert!(!r.is_empty());
    assert_eq!(r.c2_fall.values, vec![vec![0.0040]]);
    // effective load = mean of per-edge (C1+C2)/2:
    //   rise (0.0020+0.0030)/2 = 0.0025; fall (0.0018+0.0040)/2 = 0.0029
    //   -> (0.0025 + 0.0029)/2 = 0.0027
    assert!((r.effective_load() - 0.0027).abs() < 1e-9);
}

#[test]
fn receiver_load_beats_static_capacitance() {
    let lib = Lib::parse(LIB).unwrap();
    // pin with a receiver model -> Miller-aware load (0.0027), not the static 0.0019
    let inv_a = &lib.cell("INV").unwrap().pins["A"];
    assert!((inv_a.load_cap() - 0.0027).abs() < 1e-9);
    assert!(inv_a.load_cap() > inv_a.capacitance, "receiver load includes Miller > static cap");
    // pin without a receiver model -> falls back to the static capacitance
    let buf_a = &lib.cell("BUF").unwrap().pins["A"];
    assert!(buf_a.recv.is_none());
    assert!((buf_a.load_cap() - 0.0019).abs() < 1e-9);
}
