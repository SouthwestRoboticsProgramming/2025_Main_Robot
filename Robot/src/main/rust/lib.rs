use itertools::Itertools;
use jni::{
    objects::{JClass, JDoubleArray},
    sys::{jdouble, jdoubleArray, jlong},
    JNIEnv,
};
use pathfinding::math::Vec2f;

pub mod pathfinding;

fn to_handle<T>(value: T) -> jlong {
    Box::into_raw(Box::new(value)) as jlong
}

fn obstacles_from_handle<'a>(handle: jlong) -> &'a mut Vec<pathfinding::Obstacle> {
    // Here we're trusting the Java side to always give us a valid pointer
    unsafe { &mut *(handle as *mut Vec<pathfinding::Obstacle>) }
}

#[no_mangle]
pub extern "system" fn Java_com_swrobotics_lib_pathfinding_PathfindingJNI_newObstacleList<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
) -> jlong {
    to_handle(Vec::<pathfinding::Obstacle>::new())
}

#[no_mangle]
pub extern "system" fn Java_com_swrobotics_lib_pathfinding_PathfindingJNI_addCircle<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    obs_handle: jlong,
    center_x: jdouble,
    center_y: jdouble,
    radius: jdouble,
) {
    let obstacles = obstacles_from_handle(obs_handle);

    obstacles.push(pathfinding::Obstacle::Circle(pathfinding::Circle {
        position: Vec2f::new(center_x, center_y),
        radius,
    }));
}

#[no_mangle]
pub unsafe extern "system" fn Java_com_swrobotics_lib_pathfinding_PathfindingJNI_addPolygon<
    'local,
>(
    mut env: JNIEnv<'local>,
    _class: JClass<'local>,
    obs_handle: jlong,
    vertices: JDoubleArray<'local>,
) {
    let obstacles = obstacles_from_handle(obs_handle);

    let vertices_elems = env
        .get_array_elements(&vertices, jni::objects::ReleaseMode::NoCopyBack)
        .unwrap();

    let vertices = vertices_elems
        .iter()
        .tuples()
        .map(|(&x, &y)| Vec2f::new(x, y))
        .collect_vec();

    obstacles.push(pathfinding::Obstacle::Polygon(pathfinding::Polygon {
        vertices,
    }));
}

#[no_mangle]
pub extern "system" fn Java_com_swrobotics_lib_pathfinding_PathfindingJNI_buildEnvironment<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    obs_handle: jlong,
    inflate: jdouble,
) -> jlong {
    let boxed_obstacles = unsafe { Box::from_raw(obs_handle as *mut Vec<pathfinding::Obstacle>) };
    let environment = pathfinding::Environment::generate(&boxed_obstacles, inflate);
    to_handle(environment)
}

#[no_mangle]
pub extern "system" fn Java_com_swrobotics_lib_pathfinding_PathfindingJNI_findPath<'local>(
    mut env: JNIEnv<'local>,
    _class: JClass<'local>,
    env_handle: jlong,
    start_x: jdouble,
    start_y: jdouble,
    goal_x: jdouble,
    goal_y: jdouble,
) -> jdoubleArray {
    let environment = unsafe { &mut *(env_handle as *mut pathfinding::Environment) };

    let start = Vec2f::new(start_x, start_y);
    let goal = Vec2f::new(goal_x, goal_y);

    let path = environment
        .find_path(start, goal)
        .map(|result| pathfinding::to_bezier(&result, start, goal));

    match path {
        Some(path) => {
            let mut values = Vec::with_capacity(path.len() * 2);
            for vertex in path {
                values.push(vertex.x);
                values.push(vertex.y);
            }

            // Don't crash the robot code if sending result somehow fails, just
            // tell Java no path was found as a fallback
            into_java_array(&mut env, values).unwrap_or_else(|e| {
                eprintln!("Pathfinding: Failed to store result in Java array: {e:?}");
                std::ptr::null_mut()
            })
        }
        None => std::ptr::null_mut(),
    }
}

#[no_mangle]
pub extern "system" fn Java_com_swrobotics_lib_pathfinding_PathfindingJNI_getDebugData<'local>(
    mut env: JNIEnv<'local>,
    _class: JClass<'local>,
    env_handle: jlong,
) -> jdoubleArray {
    let environment = unsafe { &mut *(env_handle as *mut pathfinding::Environment) };
    let data = environment.get_debug_data();

    into_java_array(&mut env, data).unwrap()
}

#[no_mangle]
pub extern "system" fn Java_com_swrobotics_lib_pathfinding_PathfindingJNI_debugFindSafe<'local>(
    mut env: JNIEnv<'local>,
    _class: JClass<'local>,
    env_handle: jlong,
    start_x: jdouble,
    start_y: jdouble,
) -> jdoubleArray {
    let environment = unsafe { &mut *(env_handle as *mut pathfinding::Environment) };
    let data = environment.debug_find_safe(Vec2f::new(start_x, start_y));

    let mut values = Vec::with_capacity(data.len() * 2);
    for vertex in data {
        values.push(vertex.x);
        values.push(vertex.y);
    }
    into_java_array(&mut env, values).unwrap()
}

fn into_java_array(env: &mut JNIEnv, values: Vec<f64>) -> Result<jdoubleArray, jni::errors::Error> {
    let results = env.new_double_array(values.len() as i32)?;
    env.set_double_array_region(&results, 0, &values)?;
    Ok(results.into_raw())
}
