//! Loader for `tests/data/quadsim_step_refs.csv`.
//!
//! The file is self-describing: `@plant` and `@case` lines carry every
//! parameter the Python side used, so the Rust tests never hard-code a gain
//! that could silently drift from the Python.

use std::collections::HashMap;

pub const DATA: &str = include_str!("../data/quadsim_step_refs.csv");

/// One logged step: the state at `t` and the command computed from it.
#[derive(Clone, Copy, Debug)]
pub struct Row {
    pub t: f64,
    /// Setpoint (phi_d, theta_d, psi_d) [rad].
    pub sp: [f64; 3],
    /// Euler angles (phi, theta, psi) [rad].
    pub euler: [f64; 3],
    /// Body rates (p, q, r) [rad/s].
    pub rates: [f64; 3],
    /// Actual rotor speeds, QuadSim motor order [rad/s].
    pub rotor: [f64; 4],
    /// Commanded rotor speeds, QuadSim motor order [rad/s].
    pub cmd: [f64; 4],
}

pub struct Case {
    pub name: String,
    pub params: HashMap<String, f64>,
    pub rows: Vec<Row>,
}

impl Case {
    pub fn get(&self, key: &str) -> f64 {
        *self
            .params
            .get(key)
            .unwrap_or_else(|| panic!("case {}: missing parameter {key}", self.name))
    }

    pub fn dt(&self) -> f64 {
        self.get("dt")
    }
}

pub struct Fixture {
    pub plant: HashMap<String, f64>,
    pub cases: Vec<Case>,
}

impl Fixture {
    pub fn case(&self, name: &str) -> &Case {
        self.cases
            .iter()
            .find(|c| c.name == name)
            .unwrap_or_else(|| panic!("fixture has no case {name}"))
    }
}

fn parse_kv(tokens: &[&str]) -> HashMap<String, f64> {
    tokens
        .iter()
        .map(|kv| {
            let (k, v) = kv.split_once('=').expect("key=value");
            (k.to_string(), v.parse().expect("numeric value"))
        })
        .collect()
}

pub fn load() -> Fixture {
    let mut plant = HashMap::new();
    let mut cases: Vec<Case> = Vec::new();
    for line in DATA.lines().filter(|l| !l.is_empty() && !l.starts_with('#')) {
        if let Some(rest) = line.strip_prefix("@plant ") {
            let toks: Vec<&str> = rest.split_whitespace().collect();
            plant = parse_kv(&toks);
        } else if let Some(rest) = line.strip_prefix("@case ") {
            let toks: Vec<&str> = rest.split_whitespace().collect();
            cases.push(Case {
                name: toks[0].to_string(),
                params: parse_kv(&toks[1..]),
                rows: Vec::new(),
            });
        } else {
            let f: Vec<&str> = line.split(',').collect();
            assert_eq!(f.len(), 19, "malformed row: {line}");
            let v: Vec<f64> = f[1..].iter().map(|s| s.parse().unwrap()).collect();
            let case = cases.last_mut().expect("row before any @case");
            assert_eq!(case.name, f[0], "row out of its case block");
            case.rows.push(Row {
                t: v[0],
                sp: [v[1], v[2], v[3]],
                euler: [v[4], v[5], v[6]],
                rates: [v[7], v[8], v[9]],
                rotor: [v[10], v[11], v[12], v[13]],
                cmd: [v[14], v[15], v[16], v[17]],
            });
        }
    }
    Fixture { plant, cases }
}
