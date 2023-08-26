import trimesh

from db_loader import DbLoader


def foo(db_dir, config_file):
    db = DbLoader(db_dir, config_file)
    print(db)

    while True:
        oid = db.random_pick("Table")
        mesh_dir = "{}/{}.obj".format(db_dir, oid)

        print(oid)

        mesh = trimesh.load(mesh_dir)
        mesh.show()
    


if __name__ == "__main__":
    db_dir = "../../cad_dataset/rigid_object"
    config_file = db_dir + "/../cad_models.csv"

    foo(db_dir, config_file)