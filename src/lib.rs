use core::fmt;
use image::{ RgbImage, Rgb, ImageBuffer };
use std::{ops::{self, Add, Sub, Mul}, error::Error, rc::Rc, cmp::Ordering};
use nalgebra::{ Matrix3, Vector3, Point3, Matrix4};
pub struct Config {
    width: u32,
    height: u32,
    output: String,
}

impl Config {
    pub fn build(
        mut args: impl Iterator<Item = String>
    ) -> Result<Config,  CommandParsingError>{
        args.next(); // Первый аргумент - это всегда название программы (по C стандарту), поэтому пропускаем его
        
        let width = match args.next() {
            Some(arg) => {
                match arg.parse::<u32>() {
                    Ok(w) => w,
                    Err(_) => return Err(CommandParsingError::CouldNotParseWidth {})
                }
            },
            None => return Err(CommandParsingError::NoWidth {})
        };

        let height = match args.next() {
            Some(arg) => {
                match arg.parse::<u32>() {
                    Ok(w) => w,
                    Err(_) => return Err(CommandParsingError::CouldNotParseHeight {})
                }
            },
            None => return Err(CommandParsingError::NoHeight {})
        };

        let output = match args.next() {
            Some(arg) => arg,
            None => return Err(CommandParsingError::NoOutput {})
        };

        Ok(Config { width, height, output })
    }
}

#[derive(Debug)]
pub enum CommandParsingError {
    CouldNotParseWidth,
    NoWidth,
    CouldNotParseHeight,
    NoHeight,
    NoOutput
}

impl std::error::Error for CommandParsingError{}

impl fmt::Display for CommandParsingError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
      match self {
        CommandParsingError::CouldNotParseWidth => write!(f, "Ошибка парсинга аргументов: введённая ширина не является целым числом"),
        CommandParsingError::NoWidth => write!(f, "Ошибка парсинга аргументов: не введена ширина"),
        CommandParsingError::CouldNotParseHeight => write!(f, "Ошибка парсинга аргументов: введённая высота не является целым числом"),
        CommandParsingError::NoHeight => write!(f, "Ошибка парсинга аргументов: не введена высота"),
        CommandParsingError::NoOutput => write!(f, "Ошибка парсинга аргументов: не введён путь для вывода файла")
      }
    }
}

pub fn render_image(config: Config) -> Result<(), Box<dyn Error>> {
    let canvas = Canvas::new(config.width, config.height);
    
    canvas.save(config.output)?;
    Ok(())
}

struct Canvas {
    buffer: RgbImage,
}

impl Canvas {
    fn new(width: u32, height: u32) -> Canvas {
        Canvas {buffer: RgbImage::new(width, height)}
    }
    
    fn save(&self, path: String) -> Result<(), Box<dyn Error>> { 
        self.buffer.save(path)?;
        Ok(())
    }

}

pub struct Color {
    pub data: Vector3<f32>,
}

impl Color {
    pub fn new(r: f32, g: f32, b: f32) -> Color {
        Color {data: Vector3::new(r, g, b)}
    }
    pub fn hadamard_product(lhs: Color, rhs: Color) -> Color {
        let v = Vector3::new(lhs.data[0]*rhs.data[0],
                                                                 lhs.data[1]*rhs.data[1],
                                                                 lhs.data[2]*rhs.data[2]
                                                                );
        Color {data: v} 
    }
}

pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: Vector3<f32>
}

impl Ray {
    pub fn new(origin: Point3<f32>, direction: Vector3<f32>) -> Ray {
        Ray { origin, direction }
    }
    pub fn position(&self, t: f32) -> Point3<f32> {
        self.origin + self.direction * t
    }
    pub fn transform(&self, m: Matrix4<f32>) -> Ray {
        let new_dir = m.transform_vector(&self.direction);
        let new_or = m.transform_point(&self.origin);
        Ray::new(new_or, new_dir)
    }
}

pub trait Intersectable {
    fn intersect(&self, r: &Ray) -> Intersections;
}

pub struct Intersections<'a> {
    pub data: Vec<Intersection<'a>>
}

pub struct Intersection<'a> {
    pub t: f32,
    pub obj: &'a dyn Intersectable
}

impl<'a> Intersection<'a> {
    fn new(t: f32, obj: &'a dyn Intersectable) -> Intersection {
        Intersection{ t, obj }
    }
}

impl<'a> PartialEq for Intersection<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.t == other.t
    }

    fn ne(&self, other: &Self) -> bool {
        !self.eq(other)
    }
}


impl<'a> Eq for Intersection<'a> {}

impl<'a> Intersections<'a> {
    fn new(data: Vec<Intersection<'a>>) -> Intersections<'a> {
        let mut result = Intersections { data };
        result.sort();
        result
    }

    fn sort(&mut self) {
        self.data.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap());
    }

    pub fn hit(&self) -> Option<usize> {
        self.data.iter().position(|elem| elem.t >= 0.0)
    }
}

pub struct Sphere {
    pub transform: Matrix4<f32>
}

impl Intersectable for Sphere {
    fn intersect(&self, r: &Ray) -> Intersections {
        let new_r = r.transform(self.transform.try_inverse().expect("Не получилось взять обратную матрицу"));
        let sphere_to_ray = new_r.origin - Point3::new(0.0, 0.0, 0.0);
        let a = new_r.direction.dot(&new_r.direction);
        let b = 2.0 * new_r.direction.dot(&sphere_to_ray);
        let c = sphere_to_ray.dot(&sphere_to_ray) - 1.0;
        
        let discr = b*b - 4.0*a*c;
        if discr < 0.0 {
            return Intersections::new(vec![])
        }
        let mut result = Intersections::new(vec![Intersection::new((-b - discr.sqrt())/2.0/a, self), Intersection::new((-b + discr.sqrt())/2.0/a, self)]);
        result.sort();
        result
    }
}

impl Sphere {
    fn new() -> Sphere {
        Sphere { transform: Matrix4::identity() }
    }
}
   

#[cfg(test)]
mod tests {

    use nalgebra::{Translation3, Scale3};

    use super::*;
    #[test]
    fn hadamard_product_test() { 
        let c1 = Color::new(10.0, 20.0, 30.0);
        let c2 = Color::new(30.0, 20.0, 10.0); 
        let result = Color::new(300.0, 400.0, 300.0);
        assert_eq!(Color::hadamard_product(c1, c2).data, result.data);
    }

    #[test]
    fn ray_position_test() {
        let r = Ray::new(Point3::new(2.0,3.0,4.0), Vector3::new(1.0,0.0,0.0));
        assert_eq!(r.position(0.0), Point3::new(2.0,3.0,4.0));
        assert_eq!(r.position(1.0), Point3::new(3.0,3.0,4.0));
        assert_eq!(r.position(2.0), Point3::new(4.0,3.0,4.0));
        assert_eq!(r.position(-1.0), Point3::new(1.0,3.0,4.0));
    }

    #[test]
    fn intersections_test() {
        let s = Sphere::new();
        let r1 = Ray::new(Point3::new(0.0, 0.0, -5.0), Vector3::new(0.0, 0.0, 1.0));
        let r2 = Ray::new(Point3::new(0.0, 1.0, -5.0), Vector3::new(0.0, 0.0, 1.0));
        let r3 = Ray::new(Point3::new(0.0, 2.0, -5.0), Vector3::new(0.0, 0.0, 1.0));
        let r4 = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let r5 = Ray::new(Point3::new(0.0, 0.0, 5.0), Vector3::new(0.0, 0.0, 1.0));

        let intersections1 = s.intersect(&r1);
        let intersections2 = s.intersect(&r2);
        let intersections3 = s.intersect(&r3);
        let intersections4 = s.intersect(&r4);
        let intersections5 = s.intersect(&r5);

        assert!(intersections1.data == vec![Intersection::new(4.0, &s), Intersection::new(6.0, &s)]);
        assert!(intersections2.data == vec![Intersection::new(5.0, &s), Intersection::new(5.0, &s)]);
        assert!(intersections3.data == vec![]);
        assert!(intersections4.data == vec![Intersection::new(-1.0, &s), Intersection::new(1.0, &s)]);
        assert!(intersections5.data == vec![Intersection::new(-6.0, &s), Intersection::new(-4.0, &s)]);
    }

    #[test]
    fn hit_test() {
        let s = Sphere::new();
        let i1 = Intersection::new(1.0, &s);
        let i2 = Intersection::new(2.0, &s);
        let intersections1 = Intersections::new(vec![i1, i2]);
        assert_eq!(intersections1.hit().unwrap(), 0 as usize);

        let i3 = Intersection::new(-1.0, &s);
        let i4 = Intersection::new(1.0, &s);
        let intersections2 = Intersections::new(vec![i3, i4]);
        assert_eq!(intersections2.hit().unwrap(), 1 as usize);

        let i5 = Intersection::new(-1.0, &s);
        let i6 = Intersection::new(-2.0, &s);
        let intersections3 = Intersections::new(vec![i5, i6]);
        assert_eq!(intersections3.hit(), None);

        let i7 = Intersection::new(5.0, &s);
        let i8 = Intersection::new(7.0, &s);
        let i9 = Intersection::new(-3.0, &s);
        let i10 = Intersection::new(2.0, &s);
        let intersections4 = Intersections::new(vec![i7, i8, i9, i10]);
        assert_eq!(intersections4.hit().unwrap(), 1 as usize);
        assert_eq!(intersections4.data[intersections4.hit().unwrap()].t, 2.0);
    }

    #[test]
    fn transform_test() {
        let r1 = Ray::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 1.0, 0.0));
        let m1 = Translation3::new(3.0, 4.0, 5.0).to_homogeneous();
        let r2 = r1.transform(m1);
        assert_eq!(r2.origin, Point3::new(4.0, 6.0, 8.0));
        assert_eq!(r2.direction, Vector3::new(0.0, 1.0, 0.0));

        let r3 = Ray::new(Point3::new(0.0, 0.0, -5.0), Vector3::new(0.0, 0.0, 1.0));
        let mut s = Sphere::new();
        s.transform = Scale3::new(2.0,2.0,2.0).to_homogeneous();
        assert!(s.intersect(&r3).data == vec![Intersection::new(3.0, &s), Intersection::new(7.0, &s)]);
    }

}
