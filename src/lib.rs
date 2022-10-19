use core::fmt;
use image::{ RgbImage, Rgb, ImageBuffer };
use std::{ops::{self, Add, Sub, Mul}, error::Error, rc::Rc};
use nalgebra::{ Matrix3, Vector3, Point3};
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
}

pub trait Intersection {
    fn intersect(self: Rc<Self>, r: &Ray) -> Intersections;
}

pub struct Intersections {
    pub t: Vec<f32>,
    pub obj: Vec<Rc<dyn Intersection>>
}

impl Intersections {
    pub fn new(t: Vec<f32>, obj: Vec<Rc<dyn Intersection>>) -> Intersections {
        Intersections {t, obj}
    }
}

pub struct Sphere {
    pub origin: Point3<f32>,
    pub radius: f32
}

impl Intersection for Sphere {
    fn intersect(self: Rc<Self>, r: &Ray) -> Intersections {
        let sphere_to_ray = r.origin - Point3::new(0.0, 0.0, 0.0);
        let a = r.direction.dot(&r.direction);
        let b = 2.0 * r.direction.dot(&sphere_to_ray);
        let c = sphere_to_ray.dot(&sphere_to_ray) - 1.0;
        
        let discr = b*b - 4.0*a*c;
        if discr < 0.0 {
            return Intersections::new(vec![], vec![])
        }
        Intersections::new(vec![(-b - discr.sqrt())/2.0/a, (-b + discr.sqrt())/2.0/a], vec![self.clone(), self.clone()])
    }
}

impl Sphere {
    fn new(origin: Point3<f32>, radius: f32) -> Sphere {
        Sphere { origin, radius }
    }
}
   

#[cfg(test)]
mod tests {

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
        let s = Rc::new(Sphere::new(Point3::new(0.0, 0.0, 0.0), 1.0));
        let r1 = Ray::new(Point3::new(0.0, 0.0, -5.0), Vector3::new(0.0, 0.0, 1.0));
        let r1 = Ray::new(Point3::new(0.0, 1.0, -5.0), Vector3::new(0.0, 0.0, 1.0));
        let intersections1 = s.intersect(&r1);
        let intersections2 = s.intersect(&r2);
        assert_eq!(intersections1.t, vec![4.0, 6.0]);
        assert_eq!(intersections1.t, vec![4.0, 6.0]);
    }
}
