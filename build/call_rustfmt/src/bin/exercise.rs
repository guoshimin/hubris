use call_rustfmt::rustfmt;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    rustfmt("/tmp/wrong.rs")?;
    Ok(())
}
