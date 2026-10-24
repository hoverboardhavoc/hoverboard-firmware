//! What more than one test file builds on.

use hoverboard_cli::elf;

/// The guard's required symbols as names an image would carry: text symbols with the patterns'
/// text (anchors removed), and the `B` ones, which go in .bss.
pub fn all_required() -> (Vec<String>, Vec<String>) {
    let mut text = Vec::new();
    let mut bss = Vec::new();
    for p in elf::required_patterns() {
        let name = p.trim_end_matches('$').to_string();
        if let Some(b) = name.strip_prefix("B ") {
            bss.push(b.to_string());
        } else if let Some(t) = name.strip_prefix("T ") {
            text.push(t.to_string());
        } else {
            text.push(format!("_ZN{name}17h0123456789abcdefE"));
        }
    }
    (text, bss)
}
