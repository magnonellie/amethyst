//! Local transform component.
use nalgebra::{
    self as na, Isometry3, Matrix4, Quaternion, Translation3, Unit, UnitQuaternion, Vector3,
};
use serde::{
    de::{Deserialize, Deserializer},
    ser::{Serialize, Serializer},
};
use specs::prelude::{Component, DenseVecStorage, FlaggedStorage};

use orientation::Orientation;

/// Local position, rotation, and scale (from parent if it exists).
///
/// Used for rendering position and orientation.
///
/// The transforms are preformed in this order: scale, then rotation, then translation.
#[derive(Clone, Debug, PartialEq)]
pub struct Transform {
    /// Translation + rotation value
    pub iso: Isometry3<f32>,
    /// Scale vector
    pub scale: Vector3<f32>,
}

impl Transform {
    /// Makes the entity point towards `position`.
    ///
    /// `up` says which direction the entity should be 'rolled' to once it is pointing at
    /// `position`. If `up` is parallel to the direction the entity is looking, the result will be
    /// garbage.
    ///
    /// This function only works with respect to the coordinate system of its parent, so when used
    /// with an object that's not a sibling it will not do what you expect.
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// # use amethyst_core::transform::components::Transform;
    /// # use amethyst_core::nalgebra::{UnitQuaternion, Quaternion, Vector3};
    /// let mut t = Transform::default();
    /// // No rotation by default
    /// assert_eq!(*t.iso.rotation.quaternion(), Quaternion::identity());
    /// // look up with up pointing backwards
    /// t.look_at(Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    /// // our rotation should match the angle from straight ahead to straight up
    /// let rotation = UnitQuaternion::rotation_between(
    ///     &Vector3::new(0.0, 0.0, -1.0),
    ///     &Vector3::new(0.0, 1.0, 0.0),
    /// ).unwrap();
    /// assert_eq!(t.iso.rotation, rotation);
    /// ```
    // FIXME doctest
    // TODO: fix example
    #[inline]
    pub fn look_at(&mut self, target: Vector3<f32>, up: Vector3<f32>) -> &mut Self {
        self.iso.rotation =
            UnitQuaternion::look_at_rh(&(target - self.iso.translation.vector), &up);
        self
    }

    /// Returns the local object matrix for the transform.
    ///
    /// Combined with the parent's `GlobalTransform` component it gives
    /// the global (or world) matrix for the current entity.
    #[inline]
    pub fn matrix(&self) -> Matrix4<f32> {
        // This is a hot function, so manually implement the matrix-multiply to avoid a load of
        // unnecessary +0s.
        // Note: Not benchmarked

        // let quat = self.rotation.to_rotation_matrix();
        // let s = quat.matrix().as_slice();

        // let x: Vector4<f32> = Vector4::new(s[0], s[1], s[2], 0.0) * self.scale.x;
        // let y: Vector4<f32> = Vector4::new(s[3], s[4], s[5], 0.0) * self.scale.x;
        // let z: Vector4<f32> = Vector4::new(s[6], s[7], s[8], 0.0) * self.scale.x;
        // let w: Vector4<f32> = self.translation.insert_row(3, 0.0);

        // Matrix4::new(
        //     x.x, x.y, x.z, x.w, // Column 1
        //     y.x, y.y, y.z, y.w, // Column 2
        //     z.x, z.y, z.z, z.w, // Column 3
        //     w.x, w.y, w.z, w.w, // Column 4
        // )

        self.iso
            .to_homogeneous()
            .prepend_nonuniform_scaling(&self.scale)
    }

    /// Returns a reference to the translation vector.
    #[inline]
    pub fn translation(&self) -> &Vector3<f32> {
        &self.iso.translation.vector
    }

    /// Returns a mutable reference to the translation vector.
    #[inline]
    pub fn translation_mut(&mut self) -> &mut Vector3<f32> {
        &mut self.iso.translation.vector
    }

    /// Returns a reference to the rotation quaternion.
    #[inline]
    pub fn rotation(&self) -> &UnitQuaternion<f32> {
        &self.iso.rotation
    }

    /// Returns a mutable reference to the rotation quaternion.
    #[inline]
    pub fn rotation_mut(&mut self) -> &mut UnitQuaternion<f32> {
        &mut self.iso.rotation
    }

    /// Returns a reference to the isometry of the transform (translation and rotation combined).
    #[inline]
    pub fn isometry(&self) -> &Isometry3<f32> {
        &self.iso
    }

    /// Returns a mutable reference to the isometry of the transform (translation and rotation
    /// combined).
    #[inline]
    pub fn isometry_mut(&mut self) -> &mut Isometry3<f32> {
        &mut self.iso
    }

    /// Convert this transform's rotation into an Orientation, guaranteed to be 3 unit orthogonal
    /// vectors.
    pub fn orientation(&self) -> Orientation {
        Orientation::from(*self.iso.rotation.to_rotation_matrix().matrix())
    }

    /// Move relatively to its current position.
    #[inline]
    pub fn move_global(&mut self, translation: Vector3<f32>) -> &mut Self {
        self.iso.translation.vector += translation;
        self
    }

    /// Move relatively to its current position and orientation.
    ///
    /// Equivalent to rotating the translation before applying.
    #[inline]
    pub fn move_local(&mut self, translation: Vector3<f32>) -> &mut Self {
        self.iso.translation.vector += self.iso.rotation * translation;
        self
    }

    /// Move a distance along an axis.
    ///
    /// It will not move in the case where the axis is zero, for any distance.
    #[inline]
    pub fn move_along_global(&mut self, direction: Unit<Vector3<f32>>, distance: f32) -> &mut Self {
        self.iso.translation.vector += direction.as_ref() * distance;
        self
    }

    /// Move a distance along an axis.
    ///
    /// It will not move in the case where the axis is zero, for any distance.
    #[inline]
    pub fn move_along_local(&mut self, direction: Unit<Vector3<f32>>, distance: f32) -> &mut Self {
        self.iso.translation.vector += self.iso.rotation * direction.as_ref() * distance;
        self
    }

    /// Move forward relative to current position and orientation.
    #[inline]
    pub fn move_forward(&mut self, amount: f32) -> &mut Self {
        // sign is reversed because z comes towards us
        self.move_local(Vector3::new(0.0, 0.0, -amount))
    }

    /// Move backward relative to current position and orientation.
    #[inline]
    pub fn move_backward(&mut self, amount: f32) -> &mut Self {
        self.move_local(Vector3::new(0.0, 0.0, amount))
    }

    /// Move right relative to current position and orientation.
    #[inline]
    pub fn move_right(&mut self, amount: f32) -> &mut Self {
        self.move_local(Vector3::new(amount, 0.0, 0.0))
    }

    /// Move left relative to current position and orientation.
    #[inline]
    pub fn move_left(&mut self, amount: f32) -> &mut Self {
        self.move_local(Vector3::new(-amount, 0.0, 0.0))
    }

    /// Move up relative to current position and orientation.
    #[inline]
    pub fn move_up(&mut self, amount: f32) -> &mut Self {
        self.move_local(Vector3::new(0.0, amount, 0.0))
    }

    /// Move down relative to current position and orientation.
    #[inline]
    pub fn move_down(&mut self, amount: f32) -> &mut Self {
        self.move_local(Vector3::new(0.0, -amount, 0.0))
    }

    /// Adds the specified amount to the translation vectors x component.
    #[inline]
    pub fn add_x(&mut self, amount: f32) -> &mut Self {
        self.iso.translation.vector.x += amount;
        self
    }

    /// Adds the specified amount to the translation vectors y component.
    #[inline]
    pub fn add_y(&mut self, amount: f32) -> &mut Self {
        self.iso.translation.vector.y += amount;
        self
    }

    /// Adds the specified amount to the translation vectors z component.
    #[inline]
    pub fn add_z(&mut self, amount: f32) -> &mut Self {
        self.iso.translation.vector.z += amount;
        self
    }

    /// Sets the translation vectors x component to the specified value.
    #[inline]
    pub fn set_x(&mut self, value: f32) -> &mut Self {
        self.iso.translation.vector.x = value;
        self
    }

    /// Sets the translation vectors y component to the specified value.
    #[inline]
    pub fn set_y(&mut self, value: f32) -> &mut Self {
        self.iso.translation.vector.y = value;
        self
    }

    /// Sets the translation vectors z component to the specified value.
    #[inline]
    pub fn set_z(&mut self, value: f32) -> &mut Self {
        self.iso.translation.vector.z = value;
        self
    }

    /// Pitch relatively to the world.
    #[inline]
    pub fn pitch_global(&mut self, angle: f32) -> &mut Self {
        self.rotate_global(Vector3::x_axis(), angle)
    }

    /// Pitch relatively to its own rotation.
    #[inline]
    pub fn pitch_local(&mut self, angle: f32) -> &mut Self {
        self.rotate_local(Vector3::x_axis(), angle)
    }

    /// Yaw relatively to the world.
    #[inline]
    pub fn yaw_global(&mut self, angle: f32) -> &mut Self {
        self.rotate_global(Vector3::y_axis(), angle)
    }

    /// Yaw relatively to its own rotation.
    #[inline]
    pub fn yaw_local(&mut self, angle: f32) -> &mut Self {
        self.rotate_local(Vector3::y_axis(), angle)
    }

    /// Roll relatively to the world.
    #[inline]
    pub fn roll_global(&mut self, angle: f32) -> &mut Self {
        self.rotate_global(-Vector3::z_axis(), angle)
    }

    /// Roll relatively to its own rotation.
    #[inline]
    pub fn roll_local(&mut self, angle: f32) -> &mut Self {
        self.rotate_local(-Vector3::z_axis(), angle)
    }

    /// Rotate relatively to the world
    #[inline]
    pub fn rotate_global(&mut self, axis: Unit<Vector3<f32>>, angle: f32) -> &mut Self {
        let q = UnitQuaternion::from_axis_angle(&axis, angle);
        self.iso.rotation = q * self.iso.rotation;
        self
    }

    /// Rotate relatively to the current orientation
    #[inline]
    pub fn rotate_local(&mut self, axis: Unit<Vector3<f32>>, angle: f32) -> &mut Self {
        let q = UnitQuaternion::from_axis_angle(&axis, angle);
        self.iso.rotation = self.iso.rotation * q;
        self
    }

    /// Set the position.
    pub fn set_position(&mut self, position: Vector3<f32>) -> &mut Self {
        self.iso.translation.vector = position;
        self
    }

    /// Adds the specified amounts to the translation vector.
    pub fn add_xyz(&mut self, x: f32, y: f32, z: f32) -> &mut Self {
        self.add_x(x);
        self.add_y(y);
        self.add_z(z);
        self
    }

    /// Sets the specified values of the translation vector.
    pub fn set_xyz(&mut self, x: f32, y: f32, z: f32) -> &mut Self {
        self.set_position(Vector3::new(x, y, z))
    }

    /// Sets the rotation of the transform.
    pub fn set_rotation(&mut self, rotation: UnitQuaternion<f32>) -> &mut Self {
        self.iso.rotation = rotation;
        self
    }

    /// Sets the scale of the transform.
    pub fn set_scale(&mut self, x: f32, y: f32, z: f32) -> &mut Self {
        self.scale.x = x;
        self.scale.y = y;
        self.scale.z = z;
        self
    }

    /// Set the rotation using Euler x, y, z.
    ///
    /// # Arguments
    ///
    ///  - x - The angle to apply around the x axis. Also known as the pitch.
    ///  - y - The angle to apply around the y axis. Also known as the yaw.
    ///  - z - The angle to apply around the z axis. Also known as the roll.
    pub fn set_rotation_euler(&mut self, x: f32, y: f32, z: f32) -> &mut Self {
        self.iso.rotation = UnitQuaternion::from_euler_angles(z, x, y);
        self
    }

    /// Concatenates another transform onto `self`.
    pub fn concat(&mut self, other: &Self) -> &mut Self {
        self.scale.component_mul_assign(&other.scale);
        self.iso.rotation *= other.iso.rotation;
        self.iso.translation.vector +=
            self.iso.rotation * other.iso.translation.vector.component_mul(&self.scale);
        self
    }

    /// Calculates the inverse of this transform, which we need to render.
    ///
    /// We can exploit the extra information we have to perform this inverse faster than `O(n^3)`.
    pub fn view_matrix(&self) -> Matrix4<f32> {
        // todo
        self.matrix().try_inverse().unwrap()
    }
}

impl Default for Transform {
    /// The default transform does nothing when used to transform an entity.
    fn default() -> Self {
        Transform {
            iso: Isometry3::identity(),
            scale: Vector3::from_element(1.0),
        }
    }
}

impl Component for Transform {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

/// Creates a Transform using the `Vector3` as the translation vector.
impl From<Vector3<f32>> for Transform {
    fn from(translation: Vector3<f32>) -> Self {
        Transform {
            iso: Isometry3::new(translation, na::zero()),
            ..Default::default()
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename = "Transform")]
#[serde(default)]
struct SerializedTransform {
    translation: [f32; 3],
    rotation: [f32; 4],
    scale: [f32; 3],
}

impl Default for SerializedTransform {
    fn default() -> Self {
        Self {
            translation: [0.0; 3],
            rotation: [1.0, 0.0, 0.0, 0.0],
            scale: [1.0; 3]
        }
    }
}

impl<'de> Deserialize<'de> for Transform {
    fn deserialize<D>(deserializer: D) -> Result<Transform, D::Error>
    where
        D: Deserializer<'de>,
    {
        let st = SerializedTransform::deserialize(deserializer)?;

        let iso = Isometry3::from_parts(
            Translation3::new(st.translation[0], st.translation[1], st.translation[2]),
            Unit::new_normalize(Quaternion::new(
                st.rotation[0],
                st.rotation[1],
                st.rotation[2],
                st.rotation[3],
            ))
        );
        let scale = st.scale.into();
        Ok(Transform { iso, scale })
    }
}

impl Serialize for Transform {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let r = self.iso.rotation.as_ref().coords;

        let st = SerializedTransform {
            translation: self.iso.translation.vector.into(),
            rotation: [r.w, r.x, r.y, r.z],
            scale: self.scale.into(),
        };

        st.serialize(serializer)
    }
}

#[cfg(test)]
use serde_test::{assert_tokens, assert_de_tokens, Token::*};

#[test]
fn test_transform_serialization() {
    const X: f32 = 20.1;
    const Y: f32 = 21.2;
    const Z: f32 = 22.3;
    const W: f32 = 0.43274233;
    const I: f32 = 0.47601658;
    const J: f32 = 0.5192908;
    const K: f32 = 0.562565;
    const S: f32 = 10.9;
    const T: f32 = 11.8;
    const U: f32 = 12.7;
    let t1 = Transform {
        iso: Isometry3::from_parts(
            Translation3::new(X, Y, Z),
            Unit::new_unchecked(Quaternion::new(W, I, J, K))
        ),
        scale: Vector3::new(S, T, U)
    };

    assert_tokens(&t1, &[
        Struct { name: "Transform", len: 3 },
        Str("translation"),
        Tuple { len: 3 },
        F32(X),
        F32(Y),
        F32(Z),
        TupleEnd,
        Str("rotation"),
        Tuple { len: 4 },
        F32(W),
        F32(I),
        F32(J),
        F32(K),
        TupleEnd,
        Str("scale"),
        Tuple { len: 3 },
        F32(S),
        F32(T),
        F32(U),
        TupleEnd,
        StructEnd
    ]);

    // make sure that defaults are picked up!~
    let mut t2 = Transform::default();
    t2.set_x(X).set_y(Y).set_z(Z);
    assert_de_tokens(&t2, &[
        Struct { name: "Transform", len: 3 },
        Str("translation"),
        Tuple { len: 3 },
        F32(X),
        F32(Y),
        F32(Z),
        TupleEnd,
        StructEnd
    ]);

    t2 = Transform::default();
    t2.set_rotation(Unit::new_unchecked(Quaternion::new(W, I, J, K)));
    assert_de_tokens(&t2, &[
        Struct { name: "Transform", len: 3 },
        Str("rotation"),
        Tuple { len: 4 },
        F32(W),
        F32(I),
        F32(J),
        F32(K),
        TupleEnd,
        StructEnd
    ]);

    t2 = Transform::default();
    t2.set_scale(S, T, U);
    assert_de_tokens(&t2, &[
        Struct { name: "Transform", len: 3 },
        Str("scale"),
        Tuple { len: 3 },
        F32(S),
        F32(T),
        F32(U),
        TupleEnd,
        StructEnd
    ]);
}
