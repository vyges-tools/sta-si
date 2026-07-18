use vyges_sta_si::netlist::parse;

#[test]
fn parses_module_ports_instances() {
    let v = "module top ( a, b, y );\n\
             input a, b;\n\
             output y;\n\
             wire n1;\n\
             AND2 g1 ( .A(a), .B(b), .Y(n1) );\n\
             INV  g2 ( .A(n1), .Y(y) );\n\
             endmodule\n";
    let nl = parse(v).unwrap();
    assert_eq!(nl.module, "top");
    assert_eq!(nl.inputs, vec!["a", "b"]);
    assert_eq!(nl.outputs, vec!["y"]);
    assert_eq!(nl.insts.len(), 2);
    assert_eq!(nl.insts[0].cell, "AND2");
    assert_eq!(nl.insts[0].name, "g1");
    assert_eq!(
        nl.insts[0].conns,
        vec![
            ("A".into(), "a".into()),
            ("B".into(), "b".into()),
            ("Y".into(), "n1".into()),
        ]
    );
    assert_eq!(
        nl.insts[1].conns,
        vec![("A".into(), "n1".into()), ("Y".into(), "y".into())]
    );
}

#[test]
fn drops_constant_nets() {
    let v = "module m ( y ); output y; INV g ( .A(1'b0), .Y(y) ); endmodule";
    let nl = parse(v).unwrap();
    // constant tie on .A is dropped; only the .Y connection remains
    assert_eq!(nl.insts[0].conns, vec![("Y".into(), "y".into())]);
}

#[test]
fn expands_bus_ranges_and_tolerates_comments() {
    let v = "module m ( y );\n output [1:0] y; // a bus output\n wire w;\n endmodule";
    let nl = parse(v).unwrap();
    // a bus port expands to one node per bit (so bit-nets + per-bit SDC match)
    assert_eq!(nl.outputs, vec!["y[1]", "y[0]"]);
}

#[test]
fn reassembles_bit_select_connection_nets() {
    // a gate driving a bus output bit: the connection net must be "count[1]",
    // matching the bus-expanded port, so the path connects (no dangling output).
    let v = "module m ( count ); output [1:0] count;\n\
             BUF g0 ( .A(d0), .X(count[0]) ); BUF g1 ( .A(d1), .X(count[1]) ); endmodule";
    let nl = parse(v).unwrap();
    assert_eq!(nl.outputs, vec!["count[1]", "count[0]"]);
    assert!(
        nl.insts[1].conns.contains(&("X".into(), "count[1]".into())),
        "bit-select net reassembled to count[1], got {:?}",
        nl.insts[1].conns
    );
}
