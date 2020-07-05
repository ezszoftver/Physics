﻿using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK;

namespace Physics.Wpf._004
{
    class OBJLoader
    {
        public List<Vector3> vertices;
        public List<Vector3> normals;
        public List<Vector2> text_coords;

        private Dictionary<string, UInt32> material_to_id;
        private Dictionary<string, string> material_to_texture;

        public class Vertex
        {
            public Int32 id_vertex;
            public Int32 id_textcoord;
            public Int32 id_normal;

            public Vertex(Int32 id_vertex, Int32 id_textcoord, Int32 id_normal)
            {
                this.id_vertex = id_vertex;
                this.id_textcoord = id_textcoord;
                this.id_normal = id_normal;
            }
        }

        public class Material
        {
            public string texture_filename;

            public List<Vertex> indices;

            public Material(string texture_filename)
            {
                indices = new List<Vertex>();
                this.texture_filename = texture_filename;
            }
            
            public void Release()
            {
                indices.Clear();
                indices = null;
            }
        };

        public List<Material> materials;

        public OBJLoader()
        {
            vertices = new List<Vector3>();
            normals = new List<Vector3>();
            text_coords = new List<Vector2>();
            material_to_id = new Dictionary<string, UInt32>();
            material_to_texture = new Dictionary<string, string>();
            materials = new List<Material>();
        }

        public bool LoadFromFile(string directory, string filename)
        {
            materials.Add(new Material(""));

            string[] lines = File.ReadAllText(directory + filename).Split(new char[] {'\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
           
            UInt32 material_id = 0;
            string mtllib = "";

            NumberFormatInfo formatInfo = CultureInfo.CreateSpecificCulture("en-US").NumberFormat;

            foreach (string line in lines) {
                string[] words = line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

                switch (words[0]) {
                    case ("#"): {
                            break;
                        }
                    case ("v"): {
                            Vector3 v = new Vector3(float.Parse(words[1], formatInfo), float.Parse(words[2], formatInfo), float.Parse(words[3], formatInfo));
                            vertices.Add(v);
                            break;
                        }
                    case ("vn"):
                        {
                            Vector3 n = new Vector3(float.Parse(words[1], formatInfo), float.Parse(words[2], formatInfo), float.Parse(words[3], formatInfo));
                            normals.Add(n);
                            break;
                        }
                    case ("vt"): {
                            Vector2 vt = new Vector2(float.Parse(words[1], formatInfo), float.Parse(words[2], formatInfo));
                            text_coords.Add(vt);
                            break;
                        }
                    case ("mtllib"): {
                            mtllib = words[1];
                            string[] mtl_lines = File.ReadAllText(directory + @"/" + words[1]).Split(new char[] {'\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);

                            string material = "";
                            string texture_name = "";
                            bool is_save = true;

                            foreach (string mtl_line in mtl_lines) {
                                string[] mtl_words = mtl_line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                            
                                switch (mtl_words[0]) {
                                    case ("#"): {
                                            break;
                                        }
                                    case ("newmtl"): {
                                            if (is_save == false) {
                                                texture_name = "NoName.bmp";

                                                materials.Add(new Material(texture_name));
                                                material_to_texture.Add(material, texture_name);
                                                int id = material_to_id.Count;
                                                material_to_id.Add(material, (UInt32)id);

                                                material = "";
                                            }

                                            material = mtl_words[1];
                                            is_save = false;
                                            break;
                                        }
                                    case ("map_Kd"): {
                                            texture_name = mtl_words[1];

                                            materials.Add(new Material(texture_name));
                                            material_to_texture.Add(material, texture_name);
                                            int id = material_to_id.Count;
                                            material_to_id.Add(material, (UInt32)id);
                                            
                                            material = "";
                                            is_save = true;
                                            break;
                                        }
                                }
                            }

                            break;
                        }
                    case ("usemtl"): {
                            material_id = material_to_id[words[1]];
                            break;
                        }
                    case ("f"): {
                            Vertex first = null;
                            for (int i = 1; i < words.Count(); i++) {
                                string vertex = words[i];
                                string[] vnt = vertex.Split('/');

                                Int32 v_id = Int32.Parse(vnt[0]) - 1;
                                Int32 t_id = Int32.Parse(vnt[1]) - 1;
                                Int32 n_id = Int32.Parse(vnt[2]) - 1;

                                if (i <= 3)
                                {
                                    if (i == 1)
                                    {
                                        first = new Vertex(v_id, t_id, n_id);
                                        materials[(int)material_id].indices.Add(first);
                                    }
                                    else
                                    {
                                        materials[(int)material_id].indices.Add(new Vertex(v_id, t_id, n_id));
                                    }
                                }
                                else
                                {
                                    int id_last = materials[(int)material_id].indices.Count - 1;
                                    Vertex second = materials[(int)material_id].indices[id_last];
                                    // 1
                                    materials[(int)material_id].indices.Add(first);
                                    // 2
                                    materials[(int)material_id].indices.Add(second);
                                    // 3
                                    materials[(int)material_id].indices.Add(new Vertex(v_id, t_id, n_id));
                                }
                            }
                            break;
                        }
                }
            }

            return true;
        }

        public void Release()
        {
            // vertices
            vertices.Clear();
            vertices = null;

            // text coords
            text_coords.Clear();
            text_coords = null;

            normals.Clear();
            normals = null;

            // material to id
            material_to_id.Clear();
            material_to_id = null;
            // material to texture
            material_to_texture.Clear();      
            material_to_texture = null;

            // materials
            foreach (Material material in materials) {
                material.Release();
            }
            materials.Clear();
            materials = null;
        }
    }
}
