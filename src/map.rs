use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{Point3, Translation3, UnitQuaternion, Vector3};

use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::force_generator::{DefaultForceGeneratorSet, ConstantAcceleration, ForceGenerator};
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::material::{BasicMaterial, MaterialHandle};
use nphysics3d::object::ColliderDesc;
use nphysics3d::object::{BodyPartHandle, BodyStatus, DefaultBodyHandle, RigidBodyDesc};
use nphysics3d::object::{DefaultBodySet, DefaultColliderSet};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use std::path::Path;

use nphysics3d::joint::{
    BallConstraint, PinSlotConstraint, PlanarConstraint, PrismaticConstraint,
    RectangularConstraint, RevoluteConstraint, UniversalConstraint,
};
use nphysics3d::object::Ground;
use nphysics3d::object::{RigidBody, Collider, BodyHandle, DefaultColliderHandle};
use nphysics3d::solver::IntegrationParameters;
use nphysics3d::object::BodySet;
use nphysics3d::math::{Force, ForceType};
use nphysics3d::object::Body;


pub type LineT = Line;
pub struct Line {
    start: Vector3<f32>,
    end: Vector3<f32>,
}
impl Line {
    fn new(start: Vector3<f32>, end: Vector3<f32>) -> Self {
        Line { start, end }
    }
}

pub type CubeT = Cube;
pub struct Cube {
    size: Vector3<f32>,
    pos: Vector3<f32>,
    rot: Vector3<f32>,
}
impl Cube {
    fn new(size: Vector3<f32>, pos: Vector3<f32>, rot: Vector3<f32>) -> Self {
        Cube { size, pos, rot }
    }
}

pub type CylinderT = Cylinder;
pub struct Cylinder {
    r: f32,
    h: f32,
    pos: Vector3<f32>,
}
impl Cylinder {
    fn new(r: f32, h: f32, pos: Vector3<f32>) -> Self {
        Cylinder { r, h, pos }
    }
}

pub type RoadT = Road;
pub struct Road {
    position: Vector3<f32>,
    rotation: f32,
    obj: String,
    mtl: String,
}
impl Road {
    fn new(pos: &Vector3<f32>, rot: f32, objpath: &str, mtlpath: &str) -> Self {
        Road {
            position: pos.clone(),
            rotation: rot,
            obj: String::from(objpath),
            mtl: String::from(mtlpath),
        }
    }
    fn append(&self, window: &mut Window) -> SceneNode {
        let mut r = window.add_obj(
            Path::new(&self.obj),
            Path::new(&self.mtl),
            Vector3::new(0.01, 0.01, 0.01),
        );
        r.append_rotation(&UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            self.rotation,
        ));
        r.append_translation(&Translation3::new(
            self.position.x,
            self.position.y,
            self.position.z,
        ));
        r
    }
}

pub enum RotateDegree {
    Zero,
    Ninety,
    Half,
    NNinety,
}

pub enum BlockType {
    Straight1(Vector3<f32>, RotateDegree),
    // Straight2(Vector3<f32>, RotateDegree),
    Bridge1(Vector3<f32>, RotateDegree),
    Turning1(Vector3<f32>, RotateDegree),
    Crossing1(Vector3<f32>),
    // Roundabout1(Vector3<f32>, RotateDegree),
}

pub type MapT = Map;
pub struct Map {
    blocks: Vec<RoadT>,
    collisions: Vec<CubeT>,
    wires: Vec<LineT>,
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    pub bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joint_constraints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
    pub car: Vector3<f32>,
}
impl Map {
    pub fn new(car_size: Vector3<f32>) -> Self {
        Map {
            blocks: Vec::<RoadT>::new(),
            collisions: Vec::new(),
            wires: Vec::new(),
            mechanical_world: DefaultMechanicalWorld::new(Vector3::new(0.0, -9.81, 0.0)),
            geometrical_world: DefaultGeometricalWorld::new(),
            bodies: DefaultBodySet::new(),
            colliders: DefaultColliderSet::new(),
            joint_constraints: DefaultJointConstraintSet::new(),
            force_generators: DefaultForceGeneratorSet::new(),
            car: car_size,
        }
    }
    pub fn init_car(&mut self, pos: Vector3<f32>) -> DefaultBodyHandle {
        let mut rb_b = RigidBodyDesc::new().translation(pos).mass(1.0).build();
        let rb_b_h = self.bodies.insert(rb_b);
        let cl_b = ColliderDesc::new(ShapeHandle::new(Cuboid::new(self.car / 2.0)))
            //.material(MaterialHandle::new(BasicMaterial::new(0.0, 0.0)))
            .density(1.3)
            .build(BodyPartHandle(rb_b_h, 0));
        self.colliders.insert(cl_b);
        //let mut drive = Box::new(ConstantAcceleration::new(Vector3::x() * 5.0, Vector3::zeros()));
        //drive.add_body_part(BodyPartHandle(rb_b_h, 0));
        //self.force_generators.insert(drive);
        rb_b_h
    }
    pub fn add_ball(&mut self, pos: Vector3<f32>, r: f32) -> DefaultBodyHandle {
        let rb = RigidBodyDesc::new().translation(pos).mass(10.0).build();
        let rb_h = self.bodies.insert(rb);
        let cl = ColliderDesc::new(ShapeHandle::new(Ball::new(r)))
            .material(MaterialHandle::new(BasicMaterial::new(0.3, 0.5)))
            .density(1.3)
            .build(BodyPartHandle(rb_h, 0));
        self.colliders.insert(cl);
        rb_h
    }
    pub fn add_block(&mut self, block: &BlockType) {
        let degree: f32;
        let position: Vector3<f32>;
        let obj: String;
        let mtl: String;
        match block {
            BlockType::Straight1(pos, rot) => {
                position = pos.clone();
                degree = match rot {
                    RotateDegree::Zero => 0.0,
                    RotateDegree::Ninety => 3.1415 / 2.0,
                    RotateDegree::Half => 3.1415,
                    RotateDegree::NNinety => 3.1415 / -2.0,
                };
                obj = String::from("media/1/main.obj");
                mtl = String::from("media/1");
            }
            /*BlockType::Straight2(pos, rot) => {
                position = pos.clone();
                degree = match rot {
                    RotateDegree::Zero => 0.0,
                    RotateDegree::Ninety => 3.1415 / 2.0,
                    RotateDegree::Half => 3.1415,
                    RotateDegree::NNinety => 3.1415 / -2.0,
                };
                obj = String::from("media/1/main.obj");
                mtl = String::from("media/1");
            }*/
            BlockType::Bridge1(pos, rot) => {
                position = pos.clone();
                degree = match rot {
                    RotateDegree::Zero => 0.0,
                    RotateDegree::Ninety => 3.1415 / 2.0,
                    RotateDegree::Half => 3.1415,
                    RotateDegree::NNinety => 3.1415 / -2.0,
                };
                obj = String::from("media/2/main.obj");
                mtl = String::from("media/2");
            }
            BlockType::Turning1(pos, rot) => {
                position = pos.clone();
                degree = match rot {
                    RotateDegree::Zero => 0.0,
                    RotateDegree::Ninety => 3.1415 / 2.0,
                    RotateDegree::Half => 3.1415,
                    RotateDegree::NNinety => 3.1415 / -2.0,
                };
                obj = String::from("media/3/main.obj");
                mtl = String::from("media/3");
            }
            BlockType::Crossing1(pos) => {
                position = pos.clone();
                degree = 0.0;
                obj = String::from("media/4/main.obj");
                mtl = String::from("media/4");
            } /* BlockType::Roundabout1(pos, rot) => {
                  position = pos.clone();
                  degree = match rot {
                      RotateDegree::Zero => 0.0,
                      RotateDegree::Ninety => 3.1415 / 2.0,
                      RotateDegree::Half => 3.1415,
                      RotateDegree::NNinety => 3.1415 / -2.0,
                  };
                  obj = String::from("media/5/main.obj");
                  mtl = String::from("media/5");
              }*/
        }
        let road = RoadT::new(&position, degree, obj.as_str(), mtl.as_str());
        self.blocks.push(road);
    }
    pub fn add_collision(&mut self, size: Vector3<f32>, pos: Vector3<f32>, rot: Vector3<f32>) {
        self.collisions.push(CubeT::new(size, pos, rot));
    }
    pub fn add_wire(&mut self, start: Vector3<f32>, end: Vector3<f32>) {
        self.wires.push(LineT::new(start, end));
    }
    pub fn append_blocks(&self, window: &mut Window) {
        let len = self.blocks.len();
        for i in 0..len {
            self.blocks[i].append(window);
        }
    }
    pub fn append_collisions(&mut self, window: &mut Window, show: bool) {
        let len = self.collisions.len();
        for i in 0..len {
            if show {
                let mut c = window.add_cube(
                    self.collisions[i].size.x,
                    self.collisions[i].size.y,
                    self.collisions[i].size.z,
                );
                c.append_rotation(&UnitQuaternion::new(self.collisions[i].rot));
                c.append_translation(&Translation3::from(self.collisions[i].pos));
            }
            let rb = RigidBodyDesc::new().build();
            let rb_h = self.bodies.insert(rb);
            let rb_c = self.bodies.get_mut(rb_h).expect("Not Found");
            rb_c.set_status(BodyStatus::Static);
            let cl =
                ColliderDesc::new(ShapeHandle::new(Cuboid::new(self.collisions[i].size / 2.0)))
                    .material(MaterialHandle::new(BasicMaterial::new(0.3, 0.5)))
                    .set_rotation(self.collisions[i].rot)
                    .set_translation(self.collisions[i].pos)
                    .build(BodyPartHandle(rb_h, 0));
            self.colliders.insert(cl);
        }
    }
    pub fn append_wires(&self, window: &mut Window, shift: f32) {
        let len = self.wires.len();
        for i in 0..len {
            window.draw_line(
                &Point3::from(self.wires[i].start + Vector3::new(0.0, shift, 0.0)),
                &Point3::from(self.wires[i].end + Vector3::new(0.0, shift, 0.0)),
                &Point3::from(Vector3::new(0.0, 0.0, 1.0)),
            );
        }
    }
    pub fn physics_step(&mut self) {
        self.mechanical_world.step(
            &mut self.geometrical_world,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joint_constraints,
            &mut self.force_generators,
        );
    }
}
