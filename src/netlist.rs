//! Structural (gate-level) Verilog reader — the connectivity STA needs.
//!
//! v0 reads a clean structural subset: the `module` header, `input`/`output`
//! port declarations, and cell instances of the form
//! `CELL inst ( .PIN(net), .PIN(net) );`. `wire`/`reg` declarations and the
//! header port list are skipped (port direction comes from `input`/`output`);
//! `assign`/`parameter` are skipped; constant nets (`1'b0`) are dropped. Bus
//! ranges (`[7:0]`) are tolerated by skipping the range — v0 treats nets as
//! scalar. Pure std — fully unit-tested offline.

#[derive(Debug, Clone)]
pub struct Inst {
    pub cell: String,
    pub name: String,
    pub conns: Vec<(String, String)>, // (pin, net)
}

#[derive(Debug, Clone, Default)]
pub struct Netlist {
    pub module: String,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
    pub insts: Vec<Inst>,
}

#[derive(Debug)]
pub struct NetlistError(pub String);
impl std::fmt::Display for NetlistError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "netlist error: {}", self.0)
    }
}
impl std::error::Error for NetlistError {}

fn tokenize(text: &str) -> Vec<String> {
    // strip // line comments first
    let mut clean = String::with_capacity(text.len());
    for line in text.lines() {
        let l = match line.find("//") {
            Some(i) => &line[..i],
            None => line,
        };
        clean.push_str(l);
        clean.push('\n');
    }
    let mut out = Vec::new();
    let mut cur = String::new();
    let flush = |cur: &mut String, out: &mut Vec<String>| {
        if !cur.is_empty() {
            out.push(std::mem::take(cur));
        }
    };
    for ch in clean.chars() {
        match ch {
            '(' | ')' | ';' | ',' | '.' | '[' | ']' | '=' => {
                flush(&mut cur, &mut out);
                out.push(ch.to_string());
            }
            c if c.is_whitespace() => flush(&mut cur, &mut out),
            c => cur.push(c),
        }
    }
    flush(&mut cur, &mut out);
    out
}

fn is_keyword(t: &str) -> bool {
    matches!(
        t,
        "module" | "endmodule" | "input" | "output" | "inout" | "wire" | "reg"
            | "assign" | "parameter" | "localparam" | "supply0" | "supply1"
    )
}

fn is_const(net: &str) -> bool {
    net.contains('\'') // 1'b0, 1'b1, etc.
}

pub fn parse(text: &str) -> Result<Netlist, NetlistError> {
    let t = tokenize(text);
    let mut nl = Netlist::default();
    let mut i = 0;
    let n = t.len();

    // module NAME ( ... ) ;  — keep name, skip the header port list
    if let Some(p) = t.iter().position(|x| x == "module") {
        i = p + 1;
        if i < n {
            nl.module = t[i].clone();
            i += 1;
        }
        if i < n && t[i] == "(" {
            let mut depth = 0;
            while i < n {
                if t[i] == "(" {
                    depth += 1;
                } else if t[i] == ")" {
                    depth -= 1;
                    if depth == 0 {
                        i += 1;
                        break;
                    }
                }
                i += 1;
            }
        }
        if i < n && t[i] == ";" {
            i += 1;
        }
    }

    // Read a declaration's net names, **expanding bus ranges**: `output [7:0] count`
    // yields count[7]..count[0] as individual ports, so each bit matches the
    // bit-nets the gates drive (and SDC `set_output_delay [get_ports count[3]]`
    // resolves). A range stays in effect for every name in the same declaration
    // (`output [3:0] a, b;` -> a and b both expand). Scalars (no range) pass through.
    let read_names = |i: &mut usize| -> Vec<String> {
        let mut names = Vec::new();
        let mut range: Option<(i64, i64)> = None;
        while *i < n && t[*i] != ";" {
            match t[*i].as_str() {
                "," => {}
                "[" => {
                    *i += 1;
                    let mut spec = String::new();
                    while *i < n && t[*i] != "]" {
                        spec.push_str(&t[*i]);
                        *i += 1;
                    }
                    range = spec.split_once(':').and_then(|(h, l)| {
                        Some((h.trim().parse::<i64>().ok()?, l.trim().parse::<i64>().ok()?))
                    });
                }
                tok => match range {
                    Some((h, l)) => {
                        let (hi, lo) = if h >= l { (h, l) } else { (l, h) };
                        for b in (lo..=hi).rev() {
                            names.push(format!("{tok}[{b}]"));
                        }
                    }
                    None => names.push(tok.to_string()),
                },
            }
            *i += 1;
        }
        names
    };

    while i < n {
        match t[i].as_str() {
            "endmodule" => break,
            "input" => {
                i += 1;
                nl.inputs.extend(read_names(&mut i));
            }
            "output" => {
                i += 1;
                nl.outputs.extend(read_names(&mut i));
            }
            "wire" | "reg" | "inout" | "assign" | "parameter" | "localparam" | "supply0"
            | "supply1" => {
                while i < n && t[i] != ";" {
                    i += 1;
                }
            }
            ";" | ")" | "(" | "," | "." | "=" | "[" | "]" => i += 1,
            _ => {
                // candidate instance:  CELL  INST  ( .pin(net), ... ) ;
                if i + 2 < n && !is_keyword(&t[i]) && !is_keyword(&t[i + 1]) && t[i + 2] == "(" {
                    let cell = t[i].clone();
                    let name = t[i + 1].clone();
                    i += 3; // past CELL INST (
                    let mut conns = Vec::new();
                    let mut depth = 1;
                    while i < n && depth > 0 {
                        match t[i].as_str() {
                            "(" => {
                                depth += 1;
                                i += 1;
                            }
                            ")" => {
                                depth -= 1;
                                i += 1;
                            }
                            "." => {
                                // .PIN ( NET )   — NET may be a bit-select `count[7]`,
                                // which the tokenizer splits into `count [ 7 ]`; reassemble
                                // it so the connection net matches the bus-expanded port /
                                // the bit-nets other gates drive (else the node dangles).
                                let pin = t.get(i + 1).cloned().unwrap_or_default();
                                // expect '(' at i+2
                                let mut net = t.get(i + 3).cloned().unwrap_or_default();
                                if t.get(i + 4).map(|s| s.as_str()) == Some("[") {
                                    if let (Some(idx), Some("]")) =
                                        (t.get(i + 5), t.get(i + 6).map(|s| s.as_str()))
                                    {
                                        net = format!("{net}[{idx}]");
                                    }
                                }
                                if net != ")" && !is_const(&net) {
                                    conns.push((pin, net));
                                }
                                i += 1;
                            }
                            _ => i += 1,
                        }
                    }
                    nl.insts.push(Inst { cell, name, conns });
                    if i < n && t[i] == ";" {
                        i += 1;
                    }
                } else {
                    i += 1;
                }
            }
        }
    }

    if nl.module.is_empty() {
        return Err(NetlistError("no module found".into()));
    }
    Ok(nl)
}

pub fn load(path: &str) -> Result<Netlist, NetlistError> {
    let text = std::fs::read_to_string(path).map_err(|e| NetlistError(format!("{path}: {e}")))?;
    parse(&text)
}
