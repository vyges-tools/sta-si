//! The slice of `vyges-metadata.json` the constraint linter needs: **declared clock domains**.
//!
//! `interfaces[]` in that file says which clocks an IP has. `clock_domains` says how they
//! relate — one entry per domain, each naming its clock port, its reset, and the period it is
//! intended to run at. That declaration is what makes a question like *"does this SDC constrain
//! every domain the IP claims?"* answerable at all.
//!
//! It is worth being precise about why that question matters. A flow config that names one
//! `CLOCK_PORT` on a three-clock IP produces an SDC with one `create_clock`. Timing then does
//! not fail — it **passes**, reporting clean slack on the paths it was told about and saying
//! nothing about the registers it never looked at. On one real block that was 194 of 730
//! registers. The netlist alone cannot catch it, because a netlist has no opinion about how
//! many clocks *should* have been constrained. Only a declaration can.
//!
//! Deliberately a narrow reader: it takes the two fields it uses and ignores the rest of the
//! file. A metadata document that grows new sections must not become unreadable here, and this
//! engine has no business validating a schema it does not own.

use vyges_loom::json::Value;

#[derive(Debug, Clone)]
pub struct ClockDomain {
    pub name: String,
    /// The clock port driving the domain.
    pub clock: String,
    pub reset: Option<String>,
    pub period_ns: Option<f64>,
    /// `specified` | `assumed` | `derived` — whether `period_ns` is a requirement or a
    /// placeholder. A linter should not report a mismatch against an assumed period as though
    /// it were a violated specification.
    pub period_source: Option<String>,
}

#[derive(Debug, Default, Clone)]
pub struct IpMeta {
    pub domains: Vec<ClockDomain>,
    /// Groups of mutually asynchronous domains, by name.
    pub async_groups: Vec<Vec<String>>,
    /// The IP's own claim about whether its crossings carry synchronisers. `None` = unstated,
    /// which is not the same as `false`.
    pub crossings_synchronized: Option<bool>,
}

#[derive(Debug)]
pub struct MetaError(pub String);
impl std::fmt::Display for MetaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "metadata error: {}", self.0)
    }
}
impl std::error::Error for MetaError {}

fn get<'a>(v: &'a Value, key: &str) -> Option<&'a Value> {
    v.get(key)
}

fn as_str(v: Option<&Value>) -> Option<String> {
    v.and_then(Value::as_str).map(str::to_string)
}

fn as_arr(v: Option<&Value>) -> &[Value] {
    v.and_then(Value::as_array)
        .map(Vec::as_slice)
        .unwrap_or(&[])
}

impl IpMeta {
    pub fn parse(text: &str) -> Result<IpMeta, MetaError> {
        let root = vyges_loom::json::parse(text).map_err(|e| MetaError(format!("{e}")))?;
        let mut m = IpMeta::default();

        for d in as_arr(get(&root, "clock_domains")) {
            // `name` and `clock` are the schema's required pair; an entry without them is not
            // a domain we can check anything against, so skip rather than invent a name.
            let (Some(name), Some(clock)) = (as_str(get(d, "name")), as_str(get(d, "clock")))
            else {
                continue;
            };
            m.domains.push(ClockDomain {
                name,
                clock,
                reset: as_str(get(d, "reset")),
                period_ns: match get(d, "period_ns") {
                    Some(Value::Num(n)) => Some(*n),
                    _ => None,
                },
                period_source: as_str(get(d, "period_source")),
            });
        }

        if let Some(rel) = get(&root, "clock_domain_relations") {
            for g in as_arr(get(rel, "asynchronous_groups")) {
                let names: Vec<String> = as_arr(Some(g))
                    .iter()
                    .filter_map(|n| n.as_str().map(str::to_string))
                    .collect();
                if !names.is_empty() {
                    m.async_groups.push(names);
                }
            }
            m.crossings_synchronized = match get(rel, "crossings_synchronized") {
                Some(Value::Bool(b)) => Some(*b),
                _ => None,
            };
        }
        Ok(m)
    }

    pub fn load(path: &str) -> Result<IpMeta, MetaError> {
        let text = std::fs::read_to_string(path).map_err(|e| MetaError(format!("{path}: {e}")))?;
        IpMeta::parse(&text)
    }

    /// Nothing declared — the linter has no expectation to check the SDC against.
    pub fn is_empty(&self) -> bool {
        self.domains.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const META: &str = r#"{
      "name": "example",
      "interfaces": [],
      "clock_domains": [
        {"name":"core","clock":"clk_i","reset":"reset_n_i","period_ns":25.0,
         "period_source":"specified"},
        {"name":"apb","clock":"pclk_i","reset":"preset_n_i","period_ns":25.0,
         "period_source":"assumed"}
      ],
      "clock_domain_relations": {
        "asynchronous_groups": [["core"],["apb"]],
        "crossings_synchronized": false
      }
    }"#;

    #[test]
    fn reads_the_domains_and_their_relations() {
        let m = IpMeta::parse(META).unwrap();
        assert_eq!(m.domains.len(), 2);
        assert_eq!(m.domains[0].clock, "clk_i");
        assert_eq!(m.domains[1].period_source.as_deref(), Some("assumed"));
        assert_eq!(m.async_groups, vec![vec!["core"], vec!["apb"]]);
        assert_eq!(m.crossings_synchronized, Some(false));
    }

    #[test]
    fn unstated_is_not_false() {
        // A metadata file that says nothing about synchronisers must not be read as claiming
        // there are none, nor as claiming there are.
        let m = IpMeta::parse(r#"{"clock_domains":[{"name":"c","clock":"clk"}]}"#).unwrap();
        assert_eq!(m.crossings_synchronized, None);
        assert!(m.domains[0].reset.is_none());
    }

    #[test]
    fn a_metadata_file_with_no_clock_domains_is_empty_not_an_error() {
        // Most IPs will not carry the field for a while. That is not a failure.
        let m = IpMeta::parse(r#"{"name":"x","version":"1.0.0"}"#).unwrap();
        assert!(m.is_empty());
    }

    #[test]
    fn unknown_sections_are_ignored_rather_than_rejected() {
        // This engine does not own the schema and must not fail on parts of it it never reads.
        let m = IpMeta::parse(
            r#"{"clock_domains":[{"name":"c","clock":"clk"}],
                "chiplet":{"packaging":{"assembly":"3D_stacked"}},
                "some_future_section":{"nested":[1,2,3]}}"#,
        )
        .unwrap();
        assert_eq!(m.domains.len(), 1);
    }
}
