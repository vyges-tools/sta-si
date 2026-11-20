// SPDX-License-Identifier: Apache-2.0
//! Every timing-graph node's display name (`inst/pin`, or the port's name), in one text arena.
//!
//! ⛔ ONE BUFFER, not a String per pin. A `Vec<String>` held 0.82 GiB on a 5.7M-pin design, and
//! the Timer's name-to-pin map, keyed by a second copy of every name, about 1.1 GiB more. Here the
//! names are one text buffer and each id's end offset; the map holds a 64-bit hash of a name and
//! its id, and compares the text on a hit.

use std::collections::HashMap;
use std::hash::{BuildHasher, BuildHasherDefault, DefaultHasher};

/// The names, ids in the order they were added. A name may repeat.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Labels {
    text: String,
    ends: Vec<usize>,
}

impl Labels {
    pub(crate) fn len(&self) -> usize {
        self.ends.len()
    }

    #[allow(dead_code)]
    pub(crate) fn is_empty(&self) -> bool {
        self.ends.is_empty()
    }

    /// Add `name`; its id.
    pub(crate) fn push(&mut self, name: &str) -> usize {
        self.text.push_str(name);
        self.ends.push(self.text.len());
        self.ends.len() - 1
    }

    /// Add `inst/pin` without building it first; its id.
    pub(crate) fn push_pin(&mut self, inst: &str, pin: &str) -> usize {
        self.text.push_str(inst);
        self.text.push('/');
        self.text.push_str(pin);
        self.ends.push(self.text.len());
        self.ends.len() - 1
    }

    pub(crate) fn get(&self, id: usize) -> Option<&str> {
        let end = *self.ends.get(id)?;
        let start = if id == 0 { 0 } else { self.ends[id - 1] };
        Some(&self.text[start..end])
    }

    pub(crate) fn iter(&self) -> impl Iterator<Item = &str> + '_ {
        (0..self.len()).map(move |i| &self[i])
    }
}

impl std::ops::Index<usize> for Labels {
    type Output = str;
    fn index(&self, id: usize) -> &str {
        self.get(id).expect("label id in range")
    }
}

/// A name to its id. A name that repeats finds its LAST id, as the map it replaces did (built by
/// collecting `(name, id)` in id order, where a later id overwrites).
#[derive(Clone, Debug, Default)]
pub(crate) struct LabelIndex {
    // a hash to the latest id with that hash
    index: HashMap<u64, usize>,
    // the earlier ids under a hash that is taken, in id order
    more: HashMap<u64, Vec<usize>>,
}

fn hash(s: &str) -> u64 {
    BuildHasherDefault::<DefaultHasher>::default().hash_one(s)
}

impl LabelIndex {
    pub(crate) fn new(labels: &Labels) -> LabelIndex {
        let mut ix = LabelIndex { index: HashMap::with_capacity(labels.len()), more: HashMap::new() };
        for (id, name) in labels.iter().enumerate() {
            let h = hash(name);
            if let Some(prev) = ix.index.insert(h, id) {
                ix.more.entry(h).or_default().push(prev);
            }
        }
        ix
    }

    pub(crate) fn get(&self, labels: &Labels, name: &str) -> Option<usize> {
        let h = hash(name);
        let &last = self.index.get(&h)?;
        if &labels[last] == name {
            return Some(last);
        }
        self.more.get(&h)?.iter().rev().copied().find(|&i| &labels[i] == name)
    }
}

#[cfg(test)]
mod tests {
    use super::{LabelIndex, Labels};

    /// The rule: ids in the order added, each name read back as added; a name finds its id, and a
    /// name that repeats finds its LAST id (the replaced map's collect-in-order rule); an absent
    /// name finds none.
    #[test]
    fn a_name_finds_its_last_id_and_reads_back_as_added() {
        let mut l = Labels::default();
        assert_eq!(l.push("a"), 0);
        assert_eq!(l.push_pin("u1", "A"), 1);
        assert_eq!(l.push(""), 2);
        assert_eq!(l.push("u1/A"), 3);
        assert_eq!(l.iter().collect::<Vec<_>>(), ["a", "u1/A", "", "u1/A"]);
        assert_eq!(l.get(4), None);
        let ix = LabelIndex::new(&l);
        assert_eq!(ix.get(&l, "a"), Some(0));
        assert_eq!(ix.get(&l, "u1/A"), Some(3), "a repeated name finds its last id");
        assert_eq!(ix.get(&l, ""), Some(2));
        assert_eq!(ix.get(&l, "u1/Y"), None);
    }
}
