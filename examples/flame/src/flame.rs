use libm::{floorf, modff};
use nalgebra::{clamp, SMatrix, Vector2, Vector3};

type FlameCanvas = SMatrix<Vector3<u8>, 32, 32>;

fn fractf(x: f32) -> f32 {
    return modff(x).0;
}

fn hash(x: Vector2<f32>) -> Vector2<f32> {
    let k: Vector2<f32> = Vector2::new(0.3183099, 0.3678794);
    let x = x.component_mul(&k) + k.yx();
    2.0 * (16.0 * k * fractf(x.x * x.y * (x.x + x.y))).map(fractf) - Vector2::from_element(1.0)
}

fn noised(p: Vector2<f32>) -> Vector3<f32> {
    let i = p.map(floorf);
    let f = p.map(fractf);

    let u1 = f * 6.0 - Vector2::from_element(15.0);
    let u2 = f.component_mul(&u1) + Vector2::from_element(10.0);
    let u = f
        .component_mul(&f)
        .component_mul(&f)
        .component_mul(&f)
        .component_mul(&u2);

    let du1 = f - Vector2::from_element(2.0);
    let du2 = f.component_mul(&du1) + Vector2::from_element(1.0);
    let du = 30.0 * f.component_mul(&f).component_mul(&du2);

    let ga = hash(i + Vector2::new(0.0, 0.0));
    let gb = hash(i + Vector2::new(1.0, 0.0));
    let gc = hash(i + Vector2::new(0.0, 1.0));
    let gd = hash(i + Vector2::new(1.0, 1.0));

    let fa = f - Vector2::new(0.0, 0.0);
    let fb = f - Vector2::new(1.0, 0.0);
    let fc = f - Vector2::new(0.0, 1.0);
    let fd = f - Vector2::new(1.0, 1.0);

    let va = ga.dot(&fa);
    let vb = ga.dot(&fb);
    let vc = ga.dot(&fc);
    let vd = ga.dot(&fd);

    let r1 = va + u.x * (vb - va) + u.y * (vc - va) + u.x * u.y * (va - vb - vc + vd);
    let r22 = u.yx() * (va - vb - vc + vd) + Vector2::new(vb, vc) - Vector2::from_element(va);
    let r2 = ga
        + u.x * (gb - ga)
        + u.y * (gc - ga)
        + u.x * u.y * (ga - gb - gc + gd)
        + du.component_mul(&r22);

    Vector3::new(r1, r2.x, r2.y)
}

fn stepf(x: f32, l: f32) -> f32 {
    if x < l {
        1.0
    } else {
        0.0
    }
}

fn sdf_circle(p: Vector2<f32>, r: f32) -> f32 {
    p.norm() - r
}

fn shader(t: usize, x: f32, y: f32, _color: Vector3<u8>) -> Vector3<f32> {
    let uv = Vector2::new(x as f32, y as f32) / 31f32;
    let octaves = 7f32;
    let noise_amount = noised(octaves * uv + Vector2::new(0.0, t as f32)).x;
    let y_gradient = clamp(0.7 - uv.y, 0.0, 1.0) * 0.6;
    let sdf_noise = Vector2::new(0.1, 2.5 * y_gradient) * noise_amount;

    let p1 = uv - Vector2::new(0.6, 0.7) + sdf_noise;
    let p2 = uv - Vector2::new(0.6, 0.775) + sdf_noise;
    let p3 = uv - Vector2::new(0.6, 0.80) + sdf_noise;

    let amount_outer = stepf(sdf_circle(p1, 0.25), 0.0);
    let amount_inner = stepf(sdf_circle(p2, 0.175), 0.0);
    let amount_center = stepf(sdf_circle(p3, 0.075), 0.0);

    let couter: Vector3<f32> = Vector3::new(214.0, 10.0, 3.0);
    let cinner: Vector3<f32> = Vector3::new(214.0, 63.0, 2.0);
    let center: Vector3<f32> = Vector3::new(255.0, 204.0, 23.0);

    if amount_center > 0.0 {
        center
    } else if amount_inner > 0.0 {
        cinner
    } else if amount_outer > 0.0 {
        couter
    } else {
        Vector3::from_element(0f32)
    }
}

fn into_color_vec(color: Vector3<f32>) -> Vector3<u8> {
    color.map(|v| u8::try_from(floorf(v) as u32).unwrap_or(0))
}

pub fn tick(t: usize) -> FlameCanvas {
    let canvas = FlameCanvas::from_element(Vector3::repeat(0));
    canvas
        .map_with_location(|x, y, c| shader(t, x as f32, y as f32, c))
        .map(into_color_vec)
}
