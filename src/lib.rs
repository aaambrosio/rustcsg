const PLANE_EPSILON: f32 = 1e-5f32;

mod polygon_alignment {
    pub const COPLANAR: i8 = 0;
    pub const FRONT: i8 = 1;
    pub const BACK: i8 = 2;
    pub const SPANNING: i8 = 3;
}

#[derive(Clone)]
pub struct Vector {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

impl Vector {
    pub fn new(_x: f32, _y: f32, _z: f32) -> Self {
        Vector {
            x: _x,
            y: _y,
            z: _z
        }
    }

    pub fn from_array(a: [f32; 3]) -> Self {
        Vector {
            x: a[0],
            y: a[1],
            z: a[2]
        }
    }

    pub fn from_vector(v: Vector) -> Self {
        Vector {
            x: v.x,
            y: v.y,
            z: v.z
        }
    }

    pub fn negated(&self) -> Vector {
        Vector {
            x: if self.x==0f32 { 0f32 } else { -self.x },
            y: if self.y==0f32 { 0f32 } else { -self.y },
            z: if self.z==0f32 { 0f32 } else { -self.z }
        }
    }

    pub fn plus(&self, v: &Vector) -> Vector {
        Vector {
            x: self.x + v.x,
            y: self.y + v.y,
            z: self.z + v.z
        }
    }

    pub fn minus(&self, v: &Vector) -> Vector {
        Vector {
            x: self.x - v.x,
            y: self.y - v.y,
            z: self.z - v.z
        }
    }

    pub fn times(&self, t: &f32) -> Vector {
        Vector {
            x: self.x * t,
            y: self.y * t,
            z: self.z * t
        }
    }

    pub fn divided_by(&self, d: &f32) -> Vector {
        Vector {
            x: self.x / d,
            y: self.y / d,
            z: self.z / d
        }
    }

    pub fn dot(&self, v: &Vector) -> f32 {
        return self.x * v.x + self.y * v.y + self.z * v.z;
    }

    pub fn lerp(&self, v: &Vector, l: f32) -> Self {
        return self.plus(&v.minus(self).times(&l));
    }

    pub fn length(&self) -> f32 {
        return f32::sqrt(self.dot(&self)) as f32;
    }

    pub fn unit(&self) -> Vector {
        return self.divided_by(&self.length());
    }

    pub fn cross(&self, v: &Vector) -> Vector {
        Vector {
            x: self.y * v.z - self.z * v.y,
            y: self.z * v.x - self.x * v.z,
            z: self.x * v.y - self.y * v.x
        }
    }
}

#[derive(Clone)]
pub struct Vertex {
    pub position: Vector,
    pub normal: Vector
}

impl Vertex {
    pub fn new(_position: Vector, _normal: Vector) -> Self {
        Vertex {
            position: Vector::from_vector(_position),
            normal: Vector::from_vector(_normal)
        }
    }

    pub fn flip(&mut self) {
        self.normal.negated();
    }

    pub fn interpolate(&self, v: &Vertex, i: f32) -> Vertex {
        Vertex {
            position: self.position.lerp(&v.position.clone(), i),
            normal: self.normal.lerp(&v.normal.clone(), i)
        }
    }
}

#[derive(Clone)]
pub struct Plane {
    pub normal: Vector,
    pub w: f32
}

impl Plane {
    pub fn new(_normal: Vector, _w: f32) -> Self {
        Plane {
            normal: _normal,
            w: _w
        }
    }

    pub fn from_points(a: Vector, b: Vector, c: Vector) -> Plane {
        let n = b.minus(&a).cross(&c.minus(&a)).unit();

        Plane {
            normal: n.clone(),
            w: n.dot(&a)
        }
    }

    pub fn flip(&mut self) {
        self.normal = self.normal.negated();
        self.w = -self.w;
    }

    pub fn split_polygon(&self, polygon: Polygon, coplanar_front: Option<&mut Vec<Polygon>>, coplanar_back: Option<&mut Vec<Polygon>>, front: &mut Vec<Polygon>, back: &mut Vec<Polygon>) {
        let mut polygon_type = polygon_alignment::COPLANAR;
        let mut types: Vec<i8> = vec![];
        let polygon_length = polygon.vertices.len();

        for i in 0..polygon_length {
            let _t = self.normal.dot(&polygon.vertices[i].position) - self.w;
            let _type = if _t < -PLANE_EPSILON { polygon_alignment::BACK } else if _t > PLANE_EPSILON { polygon_alignment::FRONT } else { polygon_alignment::COPLANAR };
            polygon_type |= _type;
            types.push(_type);
        }

        if polygon_type == polygon_alignment::COPLANAR {
            if coplanar_front.is_some() && coplanar_back.is_some() {
                (if self.normal.dot(&polygon.plane.normal) > 0f32 { coplanar_front.unwrap() } else { coplanar_back.unwrap() }).push(polygon.clone());
            } else if coplanar_front.is_some() && coplanar_back.is_none() {
                coplanar_front.unwrap().push(polygon.clone());
            } else if coplanar_front.is_none() && coplanar_back.is_some() {
                coplanar_back.unwrap().push(polygon.clone());
            } else if coplanar_front.is_none() && coplanar_back.is_none() {
                if self.normal.dot(&polygon.plane.normal) > 0f32 {
                    front.push(polygon.clone());
                } else {
                    back.push(polygon.clone());
                }
            }
        }

        if polygon_type == polygon_alignment::FRONT {
            front.push(polygon.clone());
        }

        if polygon_type == polygon_alignment::BACK {
            back.push(polygon.clone());
        }

        if polygon_type == polygon_alignment::SPANNING {
            let mut f: Vec<Vertex> = vec![];
            let mut b: Vec<Vertex> = vec![];

            for i in 0..polygon_length {
                let j = (i + 1) % polygon_length;
                let ti = types[i];
                let tj = types[j];
                let f_vi = polygon.vertices[i].clone();
                let b_vi = polygon.vertices[i].clone();
                let vj = &polygon.vertices[j];

                if ti != polygon_alignment::BACK {
                    f.push(f_vi);
                }
                if ti != polygon_alignment::FRONT {
                    if ti != polygon_alignment::BACK {
                        b.push(b_vi.clone());
                    } else {
                        b.push(b_vi);
                    }
                }

                if (ti | tj) == polygon_alignment::SPANNING {
                    let t = (self.w - self.normal.dot(&polygon.vertices[i].position)) / self.normal.dot(&vj.position.clone().minus(&polygon.vertices[i].position));
                    let fv = &polygon.vertices[i].interpolate(&vj, t);

                    f.push(fv.clone());
                    b.push(fv.clone());
                }
            }

            if f.len() >= 3 {
                front.push(Polygon::new(f, polygon.shared));
            }

            if b.len() >= 3 {
                back.push(Polygon::new(b, polygon.shared));
            }
        }
    }
}

#[derive(Clone)]
pub struct Polygon {
    pub vertices: Vec<Vertex>,
    pub plane: Plane,
    pub shared: Option<[f32; 3]>
}

impl Polygon {
    pub fn new(_vertices: Vec<Vertex>, _shared: Option<[f32; 3]>) -> Self {
        let _plane = Plane::from_points(_vertices[0].position.clone(), _vertices[1].position.clone(), _vertices[2].position.clone());

        Polygon {
            vertices: _vertices,
            shared: _shared,
            plane: _plane
        }
    }

    pub fn flip(&mut self) {
        self.vertices.reverse();

        for v in &mut self.vertices {
            v.flip();
        }

        self.plane.flip();
    }
}

#[derive(Clone)]
pub struct Node {
    plane: Option<Plane>,
    front: Option<Box<Node>>,
    back: Option<Box<Node>>,
    polygons: Vec<Polygon>
}

impl Node {
    pub fn new(_polygons: Option<Vec<Polygon>>) -> Self {
        let mut node = Node {
            plane: None,
            front: None,
            back: None,
            polygons: vec![]
        };

        if _polygons.is_some() {
            node.build(&_polygons.unwrap());
        }

        return node;
    }

    pub fn invert(&mut self) {
        for i in 0..self.polygons.len() {
            self.polygons[i].flip();
        }

        if self.plane.is_some() {
            self.plane.as_mut().unwrap().flip();
        }

        if self.front.is_some() {
            self.front.as_mut().unwrap().invert();
        }

        if self.back.is_some() {
            self.back.as_mut().unwrap().invert();
        }

        let temp: Option<Box<Node>> = if self.front.is_some() { self.front.clone() } else { None };
        self.front = if self.back.is_some() { self.back.clone() } else { None };
        self.back = temp;
    }

    pub fn clip_polygons(&self, polygons: Vec<Polygon>) -> Vec<Polygon> {
        if self.plane.is_none() {
            return polygons.clone();
        }

        let mut front: Vec<Polygon> = vec![];
        let mut back: Vec<Polygon> = vec![];

        for i in 0..polygons.len() {
            self.plane.as_ref().unwrap().split_polygon(polygons[i].clone(), None, None, &mut front, &mut back);
        }

        if self.front.is_some() {
            front = self.front.as_ref().unwrap().clip_polygons(front);
        }
        
        if self.back.is_some() {
            back = self.back.as_ref().unwrap().clip_polygons(back);
        } else {
            back.clear();
        }

        front.extend(back);

        return front;

    }

    pub fn clip_to(&mut self, bsp: &Node) {
        self.polygons = bsp.clip_polygons(self.polygons.clone());

        if self.front.is_some() {
            self.front.as_mut().unwrap().clip_to(bsp);
        }

        if self.back.is_some() {
            self.back.as_mut().unwrap().clip_to(bsp);
        }
    }

    pub fn all_polygons(&self) -> Vec<Polygon> {
        let mut polygons: Vec<Polygon> = self.polygons.clone();

        if self.front.is_some() {
            polygons.extend(self.front.as_ref().unwrap().all_polygons());
        }

        if self.back.is_some() {
            polygons.extend(self.back.as_ref().unwrap().all_polygons());
        }

        return polygons;
    }
    
    fn build(&mut self, polygons: &Vec<Polygon>) {
        if polygons.len() == 0 {
            return;
        }

        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }

        let mut front: Vec<Polygon> = vec![];
        let mut back: Vec<Polygon> = vec![];

        for i in 0..polygons.len() {
            if self.plane.is_some() {
                self.plane.as_ref().unwrap().split_polygon(polygons[i].clone(), Some(&mut self.polygons), None, &mut front, &mut back);
            }
        }

        if front.len() > 0 {
            if self.front.is_none() {
                self.front = Some(Box::new(Node::new(None)));
            }
            
            self.front.as_mut().unwrap().build(&front);
        }

        if back.len() > 0 {
            if self.back.is_none() {
                self.back = Some(Box::new(Node::new(None)));
            }
            
            self.back.as_mut().unwrap().build(&back);
        }
    }
}

#[derive(Clone)]
pub struct CSG {
    pub polygons: Vec<Polygon>
}

impl CSG {
    pub fn from_polygons(p: Vec<Polygon>) -> CSG {
        CSG {
            polygons: p
        }
    }

    pub fn union(&self, csg: &CSG) -> CSG {
        let mut a = Node::new(Some(self.clone().polygons));
        let mut b = Node::new(Some(csg.clone().polygons));

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());

        return CSG::from_polygons(a.all_polygons());
    }

    pub fn subtract(&self, csg: &CSG) -> CSG {
        let mut a = Node::new(Some(self.clone().polygons));
        let mut b = Node::new(Some(csg.clone().polygons));

        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        a.invert();

        return CSG::from_polygons(a.all_polygons());
    }

    pub fn intersect(&self, csg: &CSG) -> CSG {
        let mut a = Node::new(Some(self.clone().polygons));
        let mut b = Node::new(Some(csg.clone().polygons));

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        return CSG::from_polygons(a.all_polygons());
    }

    pub fn triangulate(&self) -> Vec<&Vector> {
        let mut triangle_vertices: Vec<&Vector> = vec![];

        for p in &self.polygons {
            let mut i = 2;
            while i < p.vertices.len() {
                triangle_vertices.push(&p.vertices[i].position);
                triangle_vertices.push(&p.vertices[i - 1].position);
                triangle_vertices.push(&p.vertices[0].position);
                i += 1;
            }
        }

        return triangle_vertices;
    }

    pub fn cube(center: [f32; 3], radius: f32) -> CSG {
        let c = Vector::from_array(center);

        let cube_topology = [
            ([0, 4, 6, 2], [-1, 0, 0]),
            ([1, 3, 7, 5], [1, 0, 0]),
            ([0, 1, 5, 4], [0, -1, 0]),
            ([2, 6, 7, 3], [0, 1, 0]),
            ([0, 2, 3, 1], [0, 0, -1]),
            ([4, 5, 7, 6], [0, 0, 1])
        ];

        let cube_polygons: Vec<Polygon> = cube_topology.iter().map(|t| -> Polygon {
            let (position, normal) = t;

            let vertices: Vec<Vertex> = position.iter().map(|i| -> Vertex {
                let vp = Vector::new(
                    c.x + radius * (2f32 * (if !!(i & 1) != 0 { 1f32 } else { 0f32 }) - 1f32),
                    c.y + radius * (2f32 * (if !!(i & 2) != 0 { 1f32 } else { 0f32 }) - 1f32),
                    c.z + radius * (2f32 * (if !!(i & 4) != 0 { 1f32 } else { 0f32 }) - 1f32)
                );

                let vn = Vector::from_array(normal.map(|n| n as f32));

                return Vertex::new(vp, vn);
            }).collect();

            return Polygon::new(vertices, None);
        }).collect();

        return CSG::from_polygons(cube_polygons);
    }

    pub fn sphere(center: [f32; 3], radius: f32, slices: i32, stacks: i32) -> CSG {
        let c = Vector::from_array(center);

        let mut sphere_polygons: Vec<Polygon> = vec![];

        for i in 0..slices {
            for j in 0..stacks {
                let i = i as f32;
                let j = j as f32;
                let slices = slices as f32;
                let stacks = stacks as f32;

                let mut vertices: Vec<Vertex> = vec![];

                let mut fn_vertex = |mut theta: f32, mut phi: f32| {
                    theta *= std::f32::consts::PI * 2f32;
                    phi *= std::f32::consts::PI;
        
                    let dir = Vector::new(
                        theta.cos() * phi.sin(),
                        phi.cos(),
                        theta.sin() * phi.sin()
                    );
        
                    vertices.push(Vertex::new(c.plus(&dir.times(&radius)), dir));
                };

                fn_vertex(i / slices, j / stacks);
                
                if j > 0f32 {
                    fn_vertex((i + 1f32) / slices, j / stacks);
                }

                if j < stacks - 1f32 {
                    fn_vertex((i + 1f32) / slices, (j + 1f32) / stacks)
                }

                fn_vertex(i / slices, (j + 1f32) / stacks);

                sphere_polygons.push(Polygon::new(vertices, None));
            }
        }

        return CSG::from_polygons(sphere_polygons);
    }

    pub fn cylinder(radius: f32, slices: i32, start: [f32; 3], end: [f32; 3]) -> CSG {
        let s = Vector::from_array(start);
        let e = Vector::from_array(end);
        let ray = e.minus(&s);
        let r = radius;

        let axis_z = ray.unit();
        let is_y = if axis_z.y.abs() > 0.5 { 1f32 } else { 0f32 };
        let axis_x = Vector::new(is_y, -is_y, 0f32).cross(&axis_z).unit();
        
        let axis_y = axis_x.cross(&axis_z).unit();
        let v_start = Vertex::new(s.clone(), axis_z.negated());
        let v_end = Vertex::new(e, axis_z.unit());

        let mut cylinder_polygons: Vec<Polygon> = vec![];

        let point = |stack: f32, slice: f32, normal_blend: f32| -> Vertex {
            let angle = slice * std::f32::consts::PI * 2f32;
            let out = axis_x.times(&angle.cos()).plus(&axis_y.times(&angle.sin()));
            let pos = &s.clone().plus(&ray.times(&stack)).plus(&out.times(&r));
            let normal = out.times(&(1f32 - normal_blend.abs())).plus(&axis_z.times(&normal_blend));

            let p = Vector::new(
                if pos.x == -0f32 { 0f32 } else { pos.x },
                if pos.y == -0f32 { 0f32 } else { pos.y },
                if pos.z == -0f32 { 0f32 } else { pos.z }
                );

            let n = Vector::new(
                            if normal.x == -0f32 { 0f32 } else { normal.x },
                            if normal.y == -0f32 { 0f32 } else { normal.y },
                            if normal.z == -0f32 { 0f32 } else { normal.z }
                            );

            let v = Vertex::new(p, n);

            return v;
        };

        for i in 0..slices {
            let i = i as f32;
            let slices = slices as f32;

            let t0 = i / slices;
            let t1 = (i + 1f32) / slices;

            cylinder_polygons.push(Polygon::new(vec![v_start.clone(), point(0f32, t0, -1f32), point(0f32, t1, -1f32)], None));
            cylinder_polygons.push(Polygon::new(vec![point(0f32, t1, 0f32), point(0f32, t0, 0f32), point(1f32, t0, 0f32), point(1f32, t1, 0f32)], None));
            cylinder_polygons.push(Polygon::new(vec![v_end.clone(), point(1f32, t1, 1f32), point(1f32, t0, 1f32)], None));
        }

        return CSG::from_polygons(cylinder_polygons);
    }
}