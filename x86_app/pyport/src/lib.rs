use generic::atomi_proto::AtomiProto;
use generic::command_to_proto::{parse_protocol};
use pyo3::{pyfunction, pymodule, PyResult, Python, wrap_pyfunction};
use pyo3::prelude::PyModule;
use log::{error, info};

#[pyfunction]
fn command_to_binvec(line: &str) -> PyResult<Vec<u8>> {
    let proto = parse_protocol(line);
    info!("Command {:?} => Proto {:?}", line, proto);
    match postcard::to_allocvec(&proto) {
        Ok(data) => Ok(data),
        Err(err) => {
            error!("error in parse proto: {:?}", err);
            Ok(Vec::new())
        }
    }
}

#[pyfunction]
fn binvec_to_json(s: &[u8]) -> PyResult<String> {
    match postcard::from_bytes::<AtomiProto>(s) {
        Ok(proto) => Ok(format!("{:?}", proto)),
        Err(err) => {
            error!("Unknown data: {:?}, err: {:?}", s, err);
            Ok("".to_string())
        }
    }
}

#[pymodule]
fn rust_module(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(command_to_binvec, m)?)?;
    m.add_function(wrap_pyfunction!(binvec_to_json, m)?)?;
    Ok(())
}