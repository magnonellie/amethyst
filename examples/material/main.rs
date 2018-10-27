//! Displays spheres with physically based materials.

extern crate amethyst;

use amethyst::{
    assets::Loader,
    core::{nalgebra::Vector3, Transform, TransformBundle},
    prelude::*,
    renderer::*,
    utils::application_root_dir,
};

struct Example;

impl<'a, 'b> SimpleState<'a, 'b> for Example {
    fn on_start(&mut self, data: StateData<GameData>) {
        let StateData { world, .. } = data;
        let mat_defaults = world.read_resource::<MaterialDefaults>().0.clone();

        println!("Load mesh");
        let (mesh, albedo) = {
            let loader = world.read_resource::<Loader>();

            let meshes = &world.read_resource();
            let textures = &world.read_resource();
            let mesh: MeshHandle = loader.load_from_data(
                Shape::Sphere(32, 32).generate::<Vec<PosNormTangTex>>(None),
                (),
                meshes,
            );
            let albedo = loader.load_from_data([1.0, 1.0, 1.0, 1.0].into(), (), textures);

            (mesh, albedo)
        };

        println!("Create spheres");
        for i in 0..5 {
            for j in 0..5 {
                let roughness = 1.0f32 * (i as f32 / 4.0f32);
                let metallic = 1.0f32 * (j as f32 / 4.0f32);

                let metallic = [metallic, metallic, metallic, 1.0].into();
                let roughness = [roughness, roughness, roughness, 1.0].into();

                let (metallic, roughness) = {
                    let loader = world.read_resource::<Loader>();
                    let textures = &world.read_resource();

                    let metallic = loader.load_from_data(metallic, (), textures);
                    let roughness = loader.load_from_data(roughness, (), textures);

                    (metallic, roughness)
                };

                let mtl = Material {
                    albedo: albedo.clone(),
                    metallic,
                    roughness,
                    ..mat_defaults.clone()
                };

                let mut transform = Transform::default();
                transform.set_xyz(2.0f32 * (i - 2) as f32, 2.0f32 * (j - 2) as f32, 0.0);

                world
                    .create_entity()
                    .with(transform)
                    .with(mesh.clone())
                    .with(mtl)
                    .build();
            }
        }

        println!("Create lights");
        let light1: Light = PointLight {
            intensity: 6.0,
            color: [0.8, 0.0, 0.0].into(),
            ..PointLight::default()
        }.into();

        let mut light1_transform = Transform::default();
        light1_transform.set_xyz(6.0, 6.0, -6.0);

        let light2: Light = PointLight {
            intensity: 5.0,
            color: [0.0, 0.3, 0.7].into(),
            ..PointLight::default()
        }.into();

        let mut light2_transform = Transform::default();
        light2_transform.set_xyz(6.0, -6.0, -6.0);

        world
            .create_entity()
            .with(light1)
            .with(light1_transform)
            .build();

        world
            .create_entity()
            .with(light2)
            .with(light2_transform)
            .build();

        println!("Put camera");

        let mut transform = Transform::default();
        transform.set_xyz(0.0, 0.0, -12.0);
        transform.rotate_local(Vector3::y_axis(), std::f32::consts::PI);

        world
            .create_entity()
            .with(Camera::from(Projection::perspective(
                1.3,
                std::f32::consts::FRAC_PI_3,
            ))).with(transform)
            .build();
    }
}

fn main() -> amethyst::Result<()> {
    amethyst::start_logger(Default::default());

    let app_root = application_root_dir();

    let path = format!(
        "{}/examples/material/resources/display_config.ron",
        app_root
    );

    let resources = format!("{}/examples/assets/", app_root);

    let game_data = GameDataBuilder::default()
        .with_basic_renderer(path, DrawPbm::<PosNormTangTex>::new(), false)?
        .with_bundle(TransformBundle::new())?;
    let mut game = Application::new(&resources, Example, game_data)?;
    game.run();
    Ok(())
}
