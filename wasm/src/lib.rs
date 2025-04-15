use wasm_bindgen::prelude::*;
mod util;

#[wasm_bindgen]
pub fn gen(seed: i32) -> String {
    util::gen(seed as u64).to_string()
}

#[wasm_bindgen(getter_with_clone)]
pub struct Ret {
    pub score: i64,
    pub err: String,
    pub svg: String,
}

#[wasm_bindgen]
pub fn vis(_input: String, _output: String, turn: usize) -> Ret {
    let input = util::parse_input(&_input);
    let output = match util::parse_output(&input, &_output) {
        Ok(output) => output,
        Err(e) => return Ret { score: 0, err: e, svg: String::new() }
    };
    let (score, err, svg) = util::vis(&input, &output, turn);
    Ret {
        score,
        err,
        svg,
    }
}

#[wasm_bindgen]
pub fn get_max_turn(_input: String, _output: String) -> usize {
    let input = util::parse_input(&_input);
    let output = match util::parse_output(&input, &_output) {
        Ok(output) => output,
        Err(_) => return 0 // エラーの場合は 0 を返す
    };
    output.q
}
