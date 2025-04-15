#![allow(non_snake_case, unused_macros)]

use proconio::input;
use rand::prelude::*;
use rand_chacha::ChaCha20Rng;
use rand_distr::StandardNormal;
use std::collections::VecDeque;
use std::io::prelude::*;
use std::ops::RangeBounds;

// ---- ここから元の補助トレイトやマクロなど ----
pub trait SetMinMax {
    fn setmin(&mut self, v: Self) -> bool;
    fn setmax(&mut self, v: Self) -> bool;
}
impl<T> SetMinMax for T
where
    T: PartialOrd,
{
    fn setmin(&mut self, v: T) -> bool {
        *self > v && {
            *self = v;
            true
        }
    }
    fn setmax(&mut self, v: T) -> bool {
        *self < v && {
            *self = v;
            true
        }
    }
}

#[macro_export]
macro_rules! mat {
	($($e:expr),*) => { Vec::from(vec![$($e),*]) };
	($($e:expr,)*) => { Vec::from(vec![$($e),*]) };
	($e:expr; $d:expr) => { Vec::from(vec![$e; $d]) };
	($e:expr; $d:expr $(; $ds:expr)+) => { Vec::from(vec![mat![$e $(; $ds)*]; $d]) };
}

const MAX_T: usize = 5000;
// ---- ここまで元の補助トレイトやマクロなど ----

/// 入力を表す構造体
/// 
/// - eps, delta: 問題文で説明される風の影響 (標準偏差 eps) と計測誤差 (標準偏差 delta)
/// - s: ドローンの初期位置 (sx, sy)
/// - ps: 目的地の座標(10個)
/// - walls: 壁(外周含む)の座標 (x1,y1)-(x2,y2) の線分
/// - fs, alphas: 各ターンでの風・計測誤差に関する乱数列を 5000 ターン分
#[derive(Clone, Debug)]
pub struct Input {
    pub eps: f64,
    pub delta: f64,
    pub s: (i64, i64),
    pub ps: Vec<(i64, i64)>,
    pub walls: Vec<(i64, i64, i64, i64)>,
    pub alphas: Vec<f64>,
    pub fs: Vec<(i64, i64)>,
}

/// 入力構造体の表示。既定の`to_string()`呼び出しなどで利用される
impl std::fmt::Display for Input {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // 出力フォーマットは問題文記載の通り
        writeln!(f, "{} {} {:.2} {:.2}", self.ps.len(), self.walls.len(), self.eps, self.delta)?;
        writeln!(f, "{} {}", self.s.0, self.s.1)?;
        for &(px, py) in &self.ps {
            writeln!(f, "{} {}", px, py)?;
        }
        for &(x1, y1, x2, y2) in &self.walls {
            writeln!(f, "{} {} {} {}", x1, y1, x2, y2)?;
        }
        // 計測誤差 α と風 fs を 5000 ターン分出力
        for &alpha in &self.alphas {
            writeln!(f, "{}", alpha)?;
        }
        for &(fx, fy) in &self.fs {
            writeln!(f, "{} {}", fx, fy)?;
        }
        Ok(())
    }
}

/// 出力(解答)を表す構造体
/// - `q`: 実際の行動数
/// - `out`: 各ターンの操作 (`(char, i64, i64)`)
#[derive(Clone, Debug)]
pub struct Output {
    pub q: usize,
    pub out: Vec<(char, i64, i64)>,
}

/// 出力のパース
/// - 各行について、先頭が `#` であればコメントとして無視
/// - そうでなければ 'A' or 'S' と続く2つの整数を読み取り
/// - 制約を超える場合などはエラーを返す
pub fn parse_output(_input: &Input, f: &str) -> Result<Output, String> {
    let mut actions = vec![];
    for line in f.lines() {
        // '#' から始まる行はコメントとしてスキップ
        let line = line.trim_start();
        if line.starts_with('#') {
            continue;
        }
        if line.is_empty() {
            // 空行はスキップ
            continue;
        }

        let mut it = line.split_whitespace();
        let mv = match it.next() {
            Some(v) => v.chars().next().unwrap(),
            None => continue,
        };

        // 読み取り関数 (範囲チェック用)
        fn read<T: Copy + PartialOrd + std::fmt::Display + std::str::FromStr, R: RangeBounds<T>>(
            token: Option<&str>,
            range: R,
        ) -> Result<T, String> {
            if let Some(v) = token {
                if let Ok(vv) = v.parse::<T>() {
                    if !range.contains(&vv) {
                        Err(format!("Out of range: {}", vv))
                    } else {
                        Ok(vv)
                    }
                } else {
                    Err(format!("Parse error: {}", v))
                }
            } else {
                Err("Unexpected EOF".to_owned())
            }
        }

        // 2つの整数を読み取る
        let x = read(it.next(), -100000..=100000)?;
        let y = read(it.next(), -100000..=100000)?;

        // 制約チェック
        if mv != 'A' && mv != 'S' {
            return Err(format!("Invalid action: {}", mv));
        }
        // 'A' の場合、加速度ベクトルは半径 500^2 以内
        if mv == 'A' && (x * x + y * y) > 500 * 500 {
            return Err(format!("Acceleration out of range: ({}, {})", x, y));
        }
        // 'S' の場合、計測ベクトルは半径 10^10 以内＆(0,0)禁止
        if mv == 'S' && (x * x + y * y) > 10_000_000_000 {
            return Err(format!("Sensor vector out of range: ({}, {})", x, y));
        }
        if mv == 'S' && (x, y) == (0, 0) {
            return Err(format!("Sensor vector cannot be zero."));
        }
        actions.push((mv, x, y));
        if actions.len() > MAX_T {
            return Err(format!("Too many actions: {}", actions.len()));
        }
    }
    let q = actions.len();
    Ok(Output { q, out: actions })
}

/// 入力をパース
/// - 問題文の形式で与えられる入力を読み取り、Input 構造体を構築
/// - 末尾にある計測誤差αや風の加算ベクトル fs も 5000 行ずつ読み取る
pub fn parse_input(f: &str) -> Input {
    let f = proconio::source::once::OnceSource::from(f);
    input! {
        from f,
        N: usize, M: usize, eps: f64, delta: f64,
        s: (i64, i64),
        ps: [(i64, i64); N],
        walls: [(i64, i64, i64, i64); M],
        alphas: [f64; MAX_T],
        fs: [(i64, i64); MAX_T],
    }
    Input {
        eps,
        delta,
        s,
        ps,
        walls,
        alphas,
        fs,
    }
}

/// 入力生成 (サンプル)
/// - 元の `lib.rs` から `gen(seed: i32) -> String` として呼ばれる前提
/// - ここでは問題 A/B/C をランダムで生成する例
pub fn gen(seed: u64) -> Input {
    let mut rng = ChaCha20Rng::seed_from_u64(seed);

    // 問題種別をランダムに決定 (A/B/C)
    let prob_type = match rng.gen_range(0..3) {
        0 => 'A',
        1 => 'B',
        _ => 'C',
    };

    // 問題毎に壁数や eps, delta を生成
    let (M, eps, delta) = match prob_type {
        'A' => (0, rng.gen_range(1..=100) as f64, rng.gen_range(1..=20) as f64 * 0.01),
        'B' => (10, rng.gen_range(0..=1) as f64, 0.01),
        'C' => (
            rng.gen_range(1..=10) as usize,
            rng.gen_range(1..=100) as f64,
            rng.gen_range(1..=20) as f64 * 0.01,
        ),
        _ => unreachable!(),
    };

    // 初期位置 s の生成
    let s = (rng.gen_range(-99999..=99999), rng.gen_range(-99999..=99999));

    // 目的地 ps の生成: 10個
    // 既存点(s含む)とユークリッド距離が 5000^2 以下なら再生成
    let mut ps = vec![];
    while ps.len() < 10 {
        let p = (rng.gen_range(-100000..=100000), rng.gen_range(-100000..=100000));
        if ps.iter()
            .chain(std::iter::once(&s))
            .any(|&(x0, y0)| (p.0 - x0) * (p.0 - x0) + (p.1 - y0) * (p.1 - y0) <= 5000 * 5000)
        {
            continue;
        }
        ps.push(p);
    }

    // 壁の生成 (外周以外に M 本)
    let mut walls = vec![];
    for _ in 0..M {
        loop {
            let x1 = rng.gen_range(-90000..=90000);
            let y1 = rng.gen_range(-90000..=90000);
            let x2_ = x1 + rng.gen_range(-100000..=100000);
            let y2_ = y1 + rng.gen_range(-100000..=100000);

            // 壁の長さが0にならないようにチェック
            if x1 == x2_ && y1 == y2_ {
                continue;
            }
            // x2_,y2_両方が完全に範囲外なら却下
            if (x2_ < -100000 || x2_ > 100000) && (y2_ < -100000 || y2_ > 100000) {
                continue;
            }

            let x2 = x2_.clamp(-100000, 100000);
            let y2 = y2_.clamp(-100000, 100000);

            // 他の壁と交差しないか＆初期位置がこの壁上にないか
            if !intersect_with_any((x1, y1, x2, y2), &walls)
                && !point_on_line((x1, y1, x2, y2), s)
            {
                walls.push((x1, y1, x2, y2));
                break;
            }
        }
    }

    // 計測誤差α(平均1,標準偏差delta; 0以下は取り直し)と風fs(平均0,標準偏差eps)を生成
    let mut alphas = vec![];
    for _ in 0..MAX_T {
        let a = loop {
            let t = 1.0 + rng.sample::<f64, _>(StandardNormal) * delta;
            if t > 0.0 {
                break t;
            }
        };
        alphas.push(a);
    }

    let mut fs = vec![];
    for _ in 0..MAX_T {
        let fx = (rng.sample::<f64, _>(StandardNormal) * eps).round() as i64;
        let fy = (rng.sample::<f64, _>(StandardNormal) * eps).round() as i64;
        fs.push((fx, fy));
    }

    Input {
        eps,
        delta,
        s,
        ps,
        walls,
        alphas,
        fs,
    }
}

/// 壁リスト existing のいずれかと new_wall が交差するかチェック
fn intersect_with_any(new_wall: (i64, i64, i64, i64), existing: &Vec<(i64, i64, i64, i64)>) -> bool {
    let (x1, y1, x2, y2) = new_wall;
    for &(xx1, yy1, xx2, yy2) in existing {
        if crs_ss((x1, y1, x2, y2), (xx1, yy1, xx2, yy2)) {
            return true;
        }
    }
    false
}

/// 点 (sx,sy) が壁 (x1,y1)-(x2,y2) 上にあるかどうか
fn point_on_line((x1, y1, x2, y2): (i64, i64, i64, i64), (sx, sy): (i64, i64)) -> bool {
    let dx1 = x2 - x1;
    let dy1 = y2 - y1;
    let dx2 = sx - x1;
    let dy2 = sy - y1;

    // 外積が 0 & 内積チェック
    let cross = dx1 as i128 * dy2 as i128 - dy1 as i128 * dx2 as i128;
    if cross != 0 {
        return false;
    }
    let dot = dx1 as i128 * dx2 as i128 + dy1 as i128 * dy2 as i128;
    if dot < 0 {
        return false;
    }
    let len2 = (dx1 as i128).pow(2) + (dy1 as i128).pow(2);
    if dot > len2 {
        return false;
    }
    true
}

/// 整数線分 (x1,y1)-(x2,y2) と (xx1,yy1)-(xx2,yy2) が交差するか
fn crs_ss(a: (i64, i64, i64, i64), b: (i64, i64, i64, i64)) -> bool {
    let (ax1, ay1, ax2, ay2) = a;
    let (bx1, by1, bx2, by2) = b;

    fn area(x1: i64, y1: i64, x2: i64, y2: i64, x3: i64, y3: i64) -> i64 {
        (x1 - x3) * (y2 - y3) - (y1 - y3) * (x2 - x3)
    }

    if std::cmp::min(ax1, ax2) > std::cmp::max(bx1, bx2)
        || std::cmp::min(ay1, ay2) > std::cmp::max(by1, by2)
        || std::cmp::min(bx1, bx2) > std::cmp::max(ax1, ax2)
        || std::cmp::min(by1, by2) > std::cmp::max(ay1, ay2)
    {
        return false;
    }
    area(ax1, ay1, ax2, ay2, bx1, by1) * area(ax1, ay1, ax2, ay2, bx2, by2) <= 0
        && area(bx1, by1, bx2, by2, ax1, ay1) * area(bx1, by1, bx2, by2, ax2, ay2) <= 0
}

/// スコア計算 (全行動をシミュレートして最終的なスコア,エラーを返す)
pub fn compute_score(input: &Input, output: &Output) -> (i64, String) {
    let (score, err, _) = compute_score_details(input, &output.out);
    (score, err)
}

/// シミュレーションで各ターンの状態を記録するための構造体
#[derive(Clone, Debug)]
struct DroneState {
    pub pos: (f64, f64),   // 位置
    pub vel: (f64, f64),   // 速度
    pub turn: usize,       // 何ターン目か
    pub score: i64,        // 現在のスコア
    pub visited: Vec<bool> // 各目的地を訪れたかどうか
}

/// ドローン操作を管理するシミュレータ
struct Simulator {
    p: (f64, f64),
    v: (f64, f64),
    visited: Vec<bool>,
    score: i64,
    best_score: i64,
    turn_count: usize,
}

impl Simulator {
    fn new(input: &Input) -> Self {
        Simulator {
            p: (input.s.0 as f64, input.s.1 as f64),
            v: (0.0, 0.0),
            visited: vec![false; input.ps.len()],
            score: 0,
            best_score: 0,
            turn_count: 0,
        }
    }

    /// 1ターンの行動を適用
    /// - mv: 'A'(加速) or 'S'(計測)
    /// - (x, y): ベクトル
    /// - 戻り値: エラーメッセージ(あれば)
    fn apply(&mut self, input: &Input, mv: char, x: i64, y: i64) -> String {
        match mv {
            'A' => {
                // 加速
                self.v.0 += x as f64;
                self.v.1 += y as f64;
            }
            'S' => {
                // 計測 (スコア影響は特になし)
            }
            _ => return format!("Invalid action: {}", mv),
        };

        // 行動後に風の影響を受ける (turn_count番目)
        let (fx, fy) = input.fs[self.turn_count];
        self.v.0 += fx as f64;
        self.v.1 += fy as f64;

        // 毎ターン -2
        self.score -= 2;
        self.turn_count += 1;

        // 壁衝突チェック
        let new_p = (self.p.0 + self.v.0, self.p.1 + self.v.1);
        if is_collision(self.p, new_p, &input.walls) {
            // 壁衝突
            self.score -= 100;
            // 位置はそのまま、速度をゼロに戻す
            self.v = (0.0, 0.0);
        } else {
            // 衝突なしなら移動
            let old_p = self.p;
            self.p = new_p;

            // 目的地到達チェック
            for (i, &(px, py)) in input.ps.iter().enumerate() {
                if !self.visited[i] {
                    // 線分(old_p->new_p) と点(px,py) の距離 <= 1000 なら到達
                    if dist2_sp((old_p, new_p), (px as f64, py as f64)) <= 1000.0 * 1000.0 {
                        self.visited[i] = true;
                        self.score += 1000;
                    }
                }
            }
        }

        // スコアの最大値を更新
        self.best_score.setmax(self.score);

        "".to_string()
    }
}

/// 部分シミュレートして (最終スコア, エラー文, ドローン状態リスト) を返す
pub fn compute_score_details(input: &Input, out: &[(char, i64, i64)]) -> (i64, String, Vec<DroneState>) {
    let mut sim = Simulator::new(input);
    let mut states = vec![DroneState {
        pos: sim.p,
        vel: sim.v,
        turn: 0,
        score: sim.score,
        visited: sim.visited.clone(),
    }];

    let mut err_msg = String::new();
    for (turn, &(mv, x, y)) in out.iter().enumerate() {
        if turn >= MAX_T {
            err_msg = format!("Exceeded turn limit({})", MAX_T);
            break;
        }
        let e = sim.apply(input, mv, x, y);
        if !e.is_empty() {
            err_msg = e;
            break;
        }
        states.push(DroneState {
            pos: sim.p,
            vel: sim.v,
            turn: turn + 1,
            score: sim.score,
            visited: sim.visited.clone(),
        });
    }

    (sim.best_score, err_msg, states)
}

/// 外周範囲外 or walls のいずれかと交差すれば衝突
fn is_collision(p0: (f64, f64), p1: (f64, f64), walls: &Vec<(i64, i64, i64, i64)>) -> bool {
    // 外周範囲チェック
    if p1.0 < -100000.0 || p1.0 > 100000.0 || p1.1 < -100000.0 || p1.1 > 100000.0 {
        return true;
    }
    // 内壁交差チェック
    for &(x1, y1, x2, y2) in walls {
        if intersect_line_segment(p0, p1, (x1, y1), (x2, y2)) {
            return true;
        }
    }
    false
}

/// 実数線分 (p0->p1) と 整数線分 (s0->s1) の交差判定
fn intersect_line_segment(p0: (f64, f64), p1: (f64, f64), s0: (i64, i64), s1: (i64, i64)) -> bool {
    crs_ss_f(
        (p0.0, p0.1, p1.0, p1.1),
        (s0.0 as f64, s0.1 as f64, s1.0 as f64, s1.1 as f64),
    )
}

/// 実数座標版の線分交差
fn crs_ss_f(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> bool {
    let (ax1, ay1, ax2, ay2) = a;
    let (bx1, by1, bx2, by2) = b;

    if ax1.max(ax2) < bx1.min(bx2)
        || ay1.max(ay2) < by1.min(by2)
        || bx1.max(bx2) < ax1.min(ax2)
        || by1.max(by2) < ay1.min(ay2)
    {
        return false;
    }

    fn area_f(x1: f64, y1: f64, x2: f64, y2: f64, x3: f64, y3: f64) -> f64 {
        (x1 - x3) * (y2 - y3) - (y1 - y3) * (x2 - x3)
    }
    area_f(ax1, ay1, ax2, ay2, bx1, by1) * area_f(ax1, ay1, ax2, ay2, bx2, by2) <= 0.0
        && area_f(bx1, by1, bx2, by2, ax1, ay1) * area_f(bx1, by1, bx2, by2, ax2, ay2) <= 0.0
}

/// 線分 (p1->p2) と 点 q の距離^2
fn dist2_sp((p1, p2): ((f64, f64), (f64, f64)), q: (f64, f64)) -> f64 {
    let v = (p2.0 - p1.0, p2.1 - p1.1);
    let w = (q.0 - p1.0, q.1 - p1.1);
    let c1 = v.0 * w.0 + v.1 * w.1; // dot
    if c1 <= 0.0 {
        return (w.0 * w.0 + w.1 * w.1);
    }
    let c2 = v.0 * v.0 + v.1 * v.1;
    if c2 <= c1 {
        let w2 = (q.0 - p2.0, q.1 - p2.1);
        return w2.0 * w2.0 + w2.1 * w2.1;
    }
    let b = c1 / c2;
    let pb = (p1.0 + b * v.0, p1.1 + b * v.1);
    let d = (q.0 - pb.0, q.1 - pb.1);
    d.0 * d.0 + d.1 * d.1
}

// --------------------------------------------------------
// ここから可視化ロジック。lib.rs の vis() から呼ばれる想定
// --------------------------------------------------------
use svg::node::element::{Circle, Line, Style, Text};
use svg::node::element::{Rectangle};
use svg::Document;

/// **新しく追加**: 目的地の描画時に、到達済みかどうかで色を切り替える関数
fn get_goal_color(is_visited: bool) -> &'static str {
    if is_visited {
        "blue"
    } else {
        "red"
    }
}

/// 円を描画するための簡易ヘルパー
fn circle(cx: f64, cy: f64, r: f64, fill: &str) -> Circle {
    Circle::new()
        .set("cx", cx)
        .set("cy", cy)
        .set("r", r)
        .set("fill", fill)
}

/// 線分を描画するための簡易ヘルパー
fn line(x1: f64, y1: f64, x2: f64, y2: f64, stroke: &str) -> Line {
    Line::new()
        .set("x1", x1)
        .set("y1", y1)
        .set("x2", x2)
        .set("y2", y2)
        .set("stroke", stroke)
        .set("stroke-width", 2)
}

/// vis 関数
/// - turn: 部分可視化したいターン数(0〜q)
/// - return: (score, err, svg文字列)
pub fn vis(input: &Input, output: &Output, turn: usize) -> (i64, String, String) {
    // turn_clamped: 実際には output.q を超えないように
    let turn_clamped = turn.min(output.q);
    // 部分シミュレーション
    let (score, err, states) = compute_score_details(input, &output.out[..turn_clamped]);

    // 最終状態 (turn_clamped ターン適用後) の visited を参照する
    let final_state = &states[states.len() - 1];
    // SVG の描画用パラメータ
    let width = 800;
    let height = 800;
    // スケール(ざっくり。1unit=何pixelか)
    let scale = 400.0 / 100000.0;

    let conv = |(x, y): (f64, f64)| {
        (
            x * scale + (width as f64) / 2.0,
            -(y * scale) + (height as f64) / 2.0, // yは上下反転
        )
    };

    let mut doc = Document::new()
        .set("id", "vis")
        .set("viewBox", (0, 0, width, height))
        .set("width", width)
        .set("height", height)
        .set("style", "background-color:white");

    // スタイル
    doc = doc.add(Style::new("text { font-size: 12px; fill: black; }"));

    // 壁を黒線で描画
    for &(x1, y1, x2, y2) in &input.walls {
        let (sx1, sy1) = conv((x1 as f64, y1 as f64));
        let (sx2, sy2) = conv((x2 as f64, y2 as f64));
        doc = doc.add(line(sx1, sy1, sx2, sy2, "black"));
    }

    // 目的地を描画 (到達済みは青、未到達は赤)
    for (i, &(px, py)) in input.ps.iter().enumerate() {
        let (cx, cy) = conv((px as f64, py as f64));
        // 新設した関数で色を切り替え
        let color = get_goal_color(final_state.visited[i]);
        doc = doc.add(circle(cx, cy, 4.0, color));
    }

    // 各ターンのドローン位置(軌跡)を黒点で描画
    for st in &states {
        let (cx, cy) = conv(st.pos);
        doc = doc.add(circle(cx, cy, 2.0, "black"));
    }

    // スコア表示
    doc = doc.add(
        Text::new("")
            .set("x", 20)
            .set("y", 20)
            .set("fill", "black")
            .add(svg::node::Text::new(format!(
                "Score: {}, Turn: {}",
                score, turn_clamped
            ))),
    );

    (score, err, doc.to_string())
}
