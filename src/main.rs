extern crate image;
extern crate itertools;
extern crate nalgebra;
extern crate num;
extern crate rand;

mod rng;

use rng::Rng;

use itertools::Itertools;
use std::path::Path;
use rand::Rng as StdRng;
use nalgebra::{Vec2, Vec3, Pnt3, Rot3, Iso3};
use nalgebra::{Transform, Rotate, Norm, Orig};
use nalgebra::{dot};
use num::traits::{Zero, One};
use std::f32::consts::{PI, PI_2};

pub struct Ray { origin: Pnt3<f32>, direction: Vec3<f32> }

impl Ray {
    fn new(origin: Pnt3<f32>, direction: Vec3<f32>) -> Ray {
        Ray { origin: origin, direction: direction }
    }

    fn at(&self, t: f32) -> Pnt3<f32> {
        self.origin + self.direction * t
    }
}

impl nalgebra::Transform<Ray> for Iso3<f32> {
    fn transform(&self, ray: &Ray) -> Ray {
        Ray {
            origin: self.transform(&ray.origin),
            direction: self.rotate(&ray.direction),
        }
    }
    
    fn inv_transform(&self, ray: &Ray) -> Ray {
        Ray {
            origin: self.inv_transform(&ray.origin),
            direction: self.inv_rotate(&ray.direction),
        }
    }
}

struct Camera {
    origin: Pnt3<f32>,
    orientation: Rot3<f32>,
    /// Distance from image plane
    focal_length: f32,
}

impl Camera {
    fn focal_distance_from_fov(fov_degrees: f32) -> f32 {
        let half_fov = fov_degrees / 360.0 * PI;
        (0.5*PI - half_fov).tan()
    }

    fn create_ray(&self, film_pos: Vec2<f32>) -> Ray {
        let cameraspace_ray = Vec3::new(film_pos.x, film_pos.y, self.focal_length);
        Ray::new(self.origin, self.orientation.rotate(&cameraspace_ray))
    }
}

struct Intersection {
    t: f32,
    position: Pnt3<f32>,
    normal: Vec3<f32>,
    uv: Vec2<f32>,
}

trait SceneShape {
    fn has_intersection(&self, &Ray) -> Option<f32>;

    fn has_intersection_until(&self, ray: &Ray, max_t: f32) -> Option<f32> {
        match self.has_intersection(ray) {
            Some(t) if !(t > max_t) => Some(t),
            _                       => None
        }
    }

    fn intersect(&self, &Ray) -> Option<Intersection>;

    fn intersect_until(&self, ray: &Ray, max_t: f32) -> Option<Intersection> {
        match self.intersect(ray) {
            Some(i) => if !(i.t > max_t) { Some(i) } else { None },
            _       => None
        }
    }
}

struct SceneObject {
    shape: Box<SceneShape>
}

struct ShapePlane {
    transform: Iso3<f32>
}

impl SceneShape for ShapePlane {
    fn has_intersection(&self, ray: &Ray) -> Option<f32> {
        let local_ray = self.transform.inv_transform(ray);
        let t = -local_ray.origin.z / local_ray.direction.z;

        if t < 0.0 { None }
        else       { Some(t) }
    }

    fn intersect(&self, ray: &Ray) -> Option<Intersection> {
        let local_ray = self.transform.inv_transform(ray);
        let t = -local_ray.origin.z / local_ray.direction.z;

        if t < 0.0 { return None; }

        Some(Intersection {
            t: t,
            position: ray.at(t),
            uv: Vec2::zero(),
            normal: self.transform.rotate(&Vec3::z()),
        })
    }
}

struct ShapeSphere {
    transform: Iso3<f32>,
    radius: f32,
}

fn intersect_with_sphere(origin: &Pnt3<f32>, radius: f32, ray: &Ray) -> Option<f32> {
    let ref o = ray.origin - *origin;
    let ref v = ray.direction;

    let a = dot(v, v);
    let b = 2.0 * dot(o, v);
    let c = dot(o, o) - radius*radius;

    let delta = b*b - 4.0*a*c;

    if delta < 0.0 {
        None
    } else {
        Some((-b - delta.sqrt()) / (2.0*a))
    }
}

impl SceneShape for ShapeSphere {
    fn has_intersection(&self, ray: &Ray) -> Option<f32> {
        let ref local_ray = self.transform.inv_transform(ray);
        intersect_with_sphere(&Pnt3::orig(), self.radius, local_ray)
    }

    fn intersect(&self, ray: &Ray) -> Option<Intersection> {
        let ref local_ray = self.transform.inv_transform(ray);
        match intersect_with_sphere(&Pnt3::orig(), self.radius, local_ray) {
            None => None,
            Some(t) => {
                let local_pos = local_ray.at(t);
                Some(Intersection {
                    t: t,
                    position: ray.at(t),
                    uv: Vec2::zero(),
                    normal: self.transform.rotate(local_pos.as_vec()),
                })
            }
        }
    }
}

struct Scene {
    camera: Camera,
    objects: Vec<SceneObject>,
    lights: Vec<usize>,
}

impl Scene {
    fn new(camera: Camera) -> Scene {
        Scene {
            camera: camera,
            objects: Vec::new(),
            lights: Vec::new(),
        }
    }
}

fn setup_scene() -> Scene {
    let mut orientation = Rot3::one();
    orientation.look_at_z(&Vec3::y(), &Vec3::z());
    let mut s = Scene::new(Camera {
        origin: Pnt3::new(0.0, 2.0, 0.0),
        orientation: orientation,
        focal_length: Camera::focal_distance_from_fov(75.0),
    });

    s.objects.push(SceneObject {
        shape: Box::new(ShapeSphere {
            transform: Iso3::new(Vec3::new(0.0, 5.0, 0.0), Vec3::zero()),
            radius: 1.0,
        })
    });
    s.objects.push(SceneObject {
        shape: Box::new(ShapeSphere {
            transform: Iso3::new(Vec3::new(-0.5, 3.0, 1.5), Vec3::zero()),
            radius: 0.25,
        })
    });

    s.objects.push(SceneObject {
        shape: Box::new(ShapePlane {
            transform: Iso3::new(Vec3::new(0.0, 0.0, -1.0), Vec3::zero())
        })
    });

    s
}

fn find_nearest_intersection(scene: &Scene, ray: &Ray) -> Option<Intersection> {
    scene.objects.iter()
        .filter_map(|object| object.shape.intersect(ray))
        .fold1(|a, b| if a.t < b.t { a } else { b })
}

fn find_any_intersection(scene: &Scene, ray: &Ray, max_t: f32) -> bool {
    scene.objects.iter()
        .any(|object| object.shape.has_intersection_until(ray, max_t).is_some())
}

fn calc_light_incidence(scene: &Scene, rng: &mut Rng, ray: &Ray, depth: u32) -> Vec3<f32> {
    if let Some(surface_hit) = find_nearest_intersection(scene, ray) {
        Vec3::one() * -dot(&ray.direction.normalize(), &surface_hit.normal)
    } else {
        Vec3::new(0.25, 0.25, 0.75)
    }
}

fn main() {
    let image_width = 320;
    let image_height = 240;

    let image_scale = Vec2::new(2.0, -2.0) / image_height as f32;
    let image_scale_offset = Vec2::new(-(image_width as f32) / image_height as f32, 1.0);

    let mut image_data = vec![Vec3::zero(); image_width * image_height];

    let scene = setup_scene();
    let mut rng = rng::Rng::new_unseeded();

    for y in 0..image_height {
        for x in 0..image_width {
            let num_image_samples = 16;
            let mut pixel_color = Vec3::<f32>::zero();

            let pixel_pos = Vec2::new(x as f32, y as f32);

            for _sample in 0..num_image_samples {
                let sample_offset = Vec2::new(rng.gen(), rng.gen());
                let sample_pos = pixel_pos + sample_offset;

                let film_coord = sample_pos * image_scale + image_scale_offset;
                let camera_ray = scene.camera.create_ray(film_coord);

                pixel_color = pixel_color + calc_light_incidence(&scene, &mut rng, &camera_ray, 0)
                        * (1.0 / num_image_samples as f32);
            }

            image_data[y*image_width + x] = pixel_color;
        }
    }

    let output = image::ImageBuffer::from_fn(image_width as u32, image_height as u32, |x, y| {
        let pixel = image_data[(y as usize)*image_width + x as usize];
        image::Rgb([(pixel[0] * 255.0) as u8,
                    (pixel[1] * 255.0) as u8,
                    (pixel[2] * 255.0) as u8])
    });
    output.save(&Path::new("output.png")).unwrap();
}
