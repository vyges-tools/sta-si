// SPDX-License-Identifier: Apache-2.0
//! The timer reads the SPEF in its analysis form (`spef::CompactSpef`: names interned once, nodes
//! as ids). Every entry point gives the same report from either form.
//!
//! **The rule.** A node is what the String form named it: an instance pin's node from `*CONN`, a
//! top-level port's node named by the port, a port net's driver node named by the net; a name
//! the net does not hold is `NO_NAME`, which no walk finds, as an unknown name found nothing.
//! The compact walks are the String walks' code, so the reports are equal to the bit.
//!
//! **Witness.** The gate (every `examples/*/*.sta`, the sky130 correlation) is byte-identical
//! to the String-form build; making every port's node `NO_NAME` moves `thresholds/th.sta`,
//! `units/*_spef.sta` and the sky130 correlation.
use std::sync::Arc;
use vyges_sta_si::job::StaJob;
use vyges_sta_si::spef::CompactSpef;
use vyges_sta_si::sta::{analyze, analyze_compact, Timer};

const JOBS: [&str; 4] = ["examples/top/top.sta", "examples/thresholds/th.sta", "examples/units/ps_spef.sta", "examples/units/ns_spef.sta"];

#[test]
fn every_entry_point_gives_the_same_report_from_either_spef_form() {
    for j in JOBS {
        let job = StaJob::load(&format!("{}/{j}", env!("CARGO_MANIFEST_DIR"))).unwrap();
        let (nl, lib, spef) = vyges_sta_si::engine::load_inputs(&job, Default::default()).unwrap();
        let spef = spef.unwrap_or_else(|| panic!("{j}: no SPEF"));
        assert!(!spef.nets.is_empty(), "{j}: an empty SPEF compares nothing");
        let compact = CompactSpef::from_ref(&spef);
        let a = format!("{:?}", analyze(&nl, &lib, &job, Some(&spef)).unwrap());
        let b = format!("{:?}", analyze_compact(&nl, &lib, &job, Some(&compact)).unwrap());
        assert_eq!(a, b, "{j}: analyze");
        let t1 = Timer::build(&nl, &lib, &job, Some(&spef)).unwrap();
        let t2 = Timer::build_compact(&nl, &lib, &job, Some(Arc::new(CompactSpef::from_spef(spef)))).unwrap();
        assert_eq!(format!("{:?}", t1.report()), format!("{:?}", t2.report()), "{j}: Timer");
        // and the SPEF is used: without it the report differs
        let none = format!("{:?}", analyze(&nl, &lib, &job, None).unwrap());
        assert_ne!(a, none, "{j}: the SPEF moved nothing, so this compares nothing");
    }
}
