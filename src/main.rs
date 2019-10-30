extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::camera::FirstPerson;
use kiss3d::light::Light;
use kiss3d::window::Window;
use na::{Point3, Vector3, Rotation3, Isometry3};

use nphysics3d::math::{Force, ForceType};

mod map;

fn main() {
    let map1_b = [
        map::BlockType::Straight1(Vector3::new(-3.0, 0.0, 1.0), map::RotateDegree::Ninety),
        map::BlockType::Bridge1(Vector3::new(0.0, 0.0, 1.0), map::RotateDegree::Ninety),
        map::BlockType::Straight1(Vector3::new(3.0, 0.0, 1.0), map::RotateDegree::Ninety),
        map::BlockType::Straight1(Vector3::new(4.0, 0.0, 1.0), map::RotateDegree::Ninety),
        map::BlockType::Turning1(Vector3::new(5.0, 0.0, 1.0), map::RotateDegree::Ninety),
        map::BlockType::Straight1(Vector3::new(5.0, 0.0, 2.0), map::RotateDegree::Zero),
        map::BlockType::Turning1(Vector3::new(5.0, 0.0, 3.0), map::RotateDegree::Zero),
        map::BlockType::Turning1(Vector3::new(4.0, 0.0, 3.0), map::RotateDegree::NNinety),
        map::BlockType::Turning1(Vector3::new(4.0, 0.0, 2.0), map::RotateDegree::Ninety),
        map::BlockType::Turning1(Vector3::new(3.0, 0.0, 2.0), map::RotateDegree::Half),
        map::BlockType::Turning1(Vector3::new(3.0, 0.0, 3.0), map::RotateDegree::Zero),
        map::BlockType::Turning1(Vector3::new(2.0, 0.0, 3.0), map::RotateDegree::NNinety),
        map::BlockType::Turning1(Vector3::new(2.0, 0.0, 2.0), map::RotateDegree::Ninety),
        map::BlockType::Turning1(Vector3::new(1.0, 0.0, 2.0), map::RotateDegree::Half),
        map::BlockType::Turning1(Vector3::new(1.0, 0.0, 3.0), map::RotateDegree::Zero),
        map::BlockType::Turning1(Vector3::new(0.0, 0.0, 3.0), map::RotateDegree::NNinety),
        map::BlockType::Straight1(Vector3::new(0.0, 0.0, 2.0), map::RotateDegree::Zero),
        map::BlockType::Straight1(Vector3::new(0.0, 0.0, 1.0), map::RotateDegree::Zero),
        map::BlockType::Straight1(Vector3::new(0.0, 0.0, 0.0), map::RotateDegree::Zero),
        map::BlockType::Crossing1(Vector3::new(0.0, 0.0, -1.0)),
        map::BlockType::Turning1(Vector3::new(0.0, 0.0, -2.0), map::RotateDegree::Half),
        map::BlockType::Turning1(Vector3::new(1.0, 0.0, -2.0), map::RotateDegree::Ninety),
        map::BlockType::Turning1(Vector3::new(1.0, 0.0, -1.0), map::RotateDegree::Zero),
        map::BlockType::Straight1(Vector3::new(-1.0, 0.0, -1.0), map::RotateDegree::Ninety),
    ];
    let map1_w = [
        (Vector3::new(-1.6, 0.0, -1.0), Vector3::new(0.7, 0.0, -1.0)),
        (Vector3::new(0.7, 0.0, -1.0), Vector3::new(1.1, 0.0, -1.5)),
        (Vector3::new(1.1, 0.0, -1.5), Vector3::new(0.8, 0.0, -2.0)),
        (Vector3::new(0.8, 0.0, -2.0), Vector3::new(0.3, 0.0, -1.9)),
        (Vector3::new(0.3, 0.0, -1.9), Vector3::new(0.0, 0.0, -1.5)),
        (Vector3::new(0.0, 0.0, -1.5), Vector3::new(0.0, 0.0, 2.5)),
        (Vector3::new(0.0, 0.0, 2.5), Vector3::new(0.3, 0.0, 2.95)),
        (Vector3::new(0.3, 0.0, 2.95), Vector3::new(0.7, 0.0, 2.95)),
        (Vector3::new(0.7, 0.0, 2.95), Vector3::new(1.0, 0.0, 2.5)),
        (Vector3::new(1.0, 0.0, 2.5), Vector3::new(1.2, 0.0, 2.0)),
        (Vector3::new(1.2, 0.0, 2.0), Vector3::new(1.8, 0.0, 2.0)),
        (Vector3::new(1.8, 0.0, 2.0), Vector3::new(2.0, 0.0, 2.5)),
        (Vector3::new(2.0, 0.0, 2.5), Vector3::new(2.25, 0.0, 2.95)),
        (Vector3::new(2.25, 0.0, 2.95), Vector3::new(2.8, 0.0, 2.95)),
        (Vector3::new(2.8, 0.0, 2.95), Vector3::new(3.0, 0.0, 2.3)),
        (Vector3::new(3.0, 0.0, 2.3), Vector3::new(3.3, 0.0, 2.0)),
        (Vector3::new(3.3, 0.0, 2.0), Vector3::new(3.75, 0.0, 2.0)),
        (Vector3::new(3.75, 0.0, 2.0), Vector3::new(3.95, 0.0, 2.5)),
        (Vector3::new(3.95, 0.0, 2.5), Vector3::new(4.25, 0.0, 2.95)),
        (Vector3::new(4.25, 0.0, 2.95), Vector3::new(4.75, 0.0, 2.95)),
        (Vector3::new(4.75, 0.0, 2.95), Vector3::new(4.95, 0.0, 2.5)),
        (Vector3::new(4.95, 0.0, 2.5), Vector3::new(4.95, 0.0, 1.3)),
        (Vector3::new(4.95, 0.0, 1.3), Vector3::new(4.7, 0.0, 1.0)),
        (Vector3::new(4.7, 0.0, 1.0), Vector3::new(2.53, 0.0, 1.0)),
        (Vector3::new(2.53, 0.0, 1.0), Vector3::new(1.1, 0.55, 1.0)),
        (Vector3::new(1.1, 0.55, 1.0), Vector3::new(-1.1, 0.55, 1.0)),
        (Vector3::new(-1.1, 0.55, 1.0), Vector3::new(-2.53, 0.0, 1.0)),
        (Vector3::new(-2.53, 0.0, 1.0), Vector3::new(-3.53, 0.0, 1.0)),
    ];
    let map1_c = [
        (
            Vector3::new(10.0, 0.04, 7.0),
            Vector3::new(1.0, -0.01, 0.5),
            Vector3::new(0.0, 0.0, 0.0),
        ),
        (
            Vector3::new(2.2, 0.01, 1.0),
            Vector3::new(0.0, 0.5157, 1.0),
            Vector3::new(0.0, 0.0, 0.0),
        ),
        (
            Vector3::new(1.7, 0.01, 1.0),
            Vector3::new(-1.9, 0.22, 1.0),
            Vector3::new(0.0, 0.0, 3.14 / 8.9),
        ),
        (
            Vector3::new(1.7, 0.01, 1.0),
            Vector3::new(1.9, 0.22, 1.0),
            Vector3::new(0.0, 0.0, -3.14 / 9.0),
        ),
    ];

    let eye = Point3::new(5.0, 4.0, 5.0);
    let at = Point3::new(0.0, 0.0, 0.0);
    let mut cam = FirstPerson::new(eye, at);
    let mut window = Window::new("Road");
    window.set_background_color(1.0, 1.0, 1.0);
    window.set_light(Light::StickToCamera);

    let mut m = map::MapT::new(Vector3::new(0.4, 0.2, 0.3));
    for b in map1_b.iter() {
        m.add_block(b);
    }
    for w in map1_w.iter() {
        m.add_wire(w.0, w.1);
    }
    for c in map1_c.iter() {
        m.add_collision(c.0, c.1, c.2);
    }
    m.append_blocks(&mut window);
    m.append_collisions(&mut window, false);
    //m.append_collisions(&mut window, true);

    let mut ball = window.add_sphere(0.2);
    ball.set_color(1.0, 0.0, 0.0);
    let mut car = window.add_cube(m.car.x, m.car.y, m.car.z);
    car.set_color(0.0, 1.0, 0.0);
    let c_h = m.init_car(Vector3::new(-1.0, 1.0, 1.0));
    let b_h = m.add_ball(Vector3::new(1.3, 5.0, 1.0), 0.2);

    while !window.should_close() {
        let rb = m.bodies.rigid_body(b_h).expect("Not Found");
        ball.set_local_translation(rb.position().translation);
        let rb = m.bodies.get_mut(c_h).expect("Not Found");
        rb.apply_force(0, &Force::new(Vector3::x() * 6.0, Vector3::zeros()), ForceType::Force, true);
        let rb = m.bodies.rigid_body(c_h).expect("Not Found");
        car.set_local_rotation(rb.position().rotation);
        car.set_local_translation(rb.position().translation);

        let pos = rb.position().translation.clone();
        let (x,y,z) = rb.position().rotation.euler_angles();
        let rot = Rotation3::new(Vector3::new(x,y,z));
        pos.append_rotation(rot);
        //let mut cam = FirstPerson::new(n_eye, n_at);

        m.append_wires(&mut window, 0.1);
        m.physics_step();
        window.render_with_camera(&mut cam);
    }
}
