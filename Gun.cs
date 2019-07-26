namespace Assets.Scripts
{
    using System.Collections.Generic;
    using UnityEngine;
    using UnityEditor;
    using UnityEngine;

    
    public class Gun : MonoBehaviour
    {
        public float radius;
        public float ray_distance;
        public PolygonCollider2D coll;
        List<GameObject> object_list = new List<GameObject>();
        Collider2D[] potential_collisions;
        Collider2D[] colliders;
        List<Vector3> collisions = new List<Vector3>();
        
        private void Start()
        {
            colliders = FindObjectsOfType<Collider2D>();
            foreach(Collider2D collider in colliders){
                
                if (collider.GetComponent<MeshFilter>() != null)
                {
                    object_list.Add(collider.gameObject);
                }
            }

        }

        public Camera camera;
        public GameObject previous_tracer = null;
        public void FixedUpdate()
        {
            fire_ray();
            if (Input.GetMouseButtonDown(0))
            {
                Shoot();
            }
            if (Input.GetAxis("Mouse X") > 0 || Input.GetAxis("Mouse X") < 0)
            {
                //Destroy(previous_tracer);
                //predict_path();
            }
            
        }

        public void fire_ray(){
            
            Vector3 mouse_pos = camera.ScreenToWorldPoint(Input.mousePosition);
            Vector3 target = (transform.position - mouse_pos);
            float projectile_azimuth = target.y / target.x;
            Vector2[] points = { new Vector2(-radius, 0),
                                new Vector2(radius,0),
                                new Vector2(radius,-ray_distance),
                                new Vector2(-radius,-ray_distance)
                                };
            coll.SetPath(0,points);
            Vector3 final_collision_point = new Vector3(0,0,0);
            foreach(Collider2D collider2d in colliders){
                if (collider2d.gameObject == gameObject){
                    continue;
                }
                if (!coll.IsTouching(collider2d)){
                    continue;
                }
                Vector3[] vertices = collider2d.gameObject.GetComponent<MeshFilter>().mesh.vertices;
                for (int i = 0;i< vertices.Length;i++){
                    if (vertices[i] == vertices[(i + 1) % vertices.Length]) {
                        continue;
                    }
                    float x0 = vertices[i].x;
                    float y0 = vertices[i].y;
                    float x1 = vertices[(i + 1) % vertices.Length].x;
                    float y1 = vertices[(i + 1) % vertices.Length].y;
                    float target_azimuth = (y1 - y0) / (x1 - x0);
                    Vector2 p1 = collider2d.gameObject.transform.TransformPoint(new Vector2(x0, y0));
                    Vector2 p2 = collider2d.gameObject.transform.TransformPoint(new Vector2(x1, y1));
                    Debug.DrawLine(p1, p2, Color.cyan);
                    p1 = transform.InverseTransformPoint(p1);
                    p2 = transform.InverseTransformPoint(p2);
                    if (p1.x < -radius && p2.x < -radius) {
                        // TODO: ACCOUNT FOR COORDINATES ABOVE THE TANK
                        continue;
                    }
                    if (p1.x > radius && p2.x > radius) {
                        continue;
                    }
                    //else if (target_azimuth == projectile_azimuth){
                    //    continue;
                    //    // TODO: ACCOUNT FOR BOTH VERTICES PARALLEL TO TRAJECTORY AND WITHIN RADIUS
                    //}
                    float slope = (p2.y - p1.y) / (p2.x - p1.x);
                    float y = p2.x * slope - p2.y;

                    float vertical_offset = y0 - target_azimuth * x0;
                    Vector3 collision_point = new Vector3(0, y, 0);
                    collision_point.x = collision_point.x * -1;
                    collision_point.y = collision_point.y * -1;
                    collision_point = transform.TransformPoint(collision_point);
                    // TODO: CODE FOR AZIMUTH BEING INF WHEN FIRING STRAIGHT UP OR DOWN
                    Vector3 edge = new Vector3(x1 - x0, y1 - y0);
                    float projectile_offset = radius / Mathf.Sin(Vector2.Angle(target, edge));
                    float centerx = (collision_point.x - Mathf.Cos(target.x) * projectile_offset);
                    float centery = (collision_point.y - Mathf.Sin(target.y) * projectile_offset);
                    if (final_collision_point == new Vector3(0,0,0)){
                        final_collision_point = new Vector3(centerx,centery);
                    }
                    else if (final_collision_point.magnitude > collision_point.magnitude){
                        final_collision_point = new Vector3(centerx, centery);
                    }
                }   
            }
            Debug.Log(final_collision_point);
            Debug.DrawLine(transform.position, final_collision_point,Color.red);
        }




        public Transform Barrel;
        public float BallSpeed = 2000;

        public void Shoot()
        {
            var ball = (GameObject)Instantiate(Resources.Load("Ball"));
            ball.transform.position = Barrel.position;
            var e = transform.rotation.eulerAngles;
            ball.GetComponent<Rigidbody2D>().AddForce(transform.up * 2 * -BallSpeed);
        }
            // WHY ARE THESE TWO DIFFERENT??????

        public void predict_path(){
            var ball = (GameObject)Instantiate(Resources.Load("tracer_ball"));
            ball.transform.position = Barrel.position;
            var e = transform.rotation.eulerAngles;
            ball.GetComponent<Rigidbody2D>().AddForce(transform.up * 2 * -BallSpeed);
            previous_tracer = ball;
        }
    }
}
