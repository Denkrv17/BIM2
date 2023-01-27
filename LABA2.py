import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import NemAll_Python_Utility as AllplanUtil
import GeometryValidate as GeometryValidate
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties


print('Load Chamfer.py')


def check_allplan_version(build_ele, version):
    del build_ele
    del version
    return True


def create_element(build_ele, doc):
    element = Chamfer(doc)
    return element.create(build_ele)


def move_handle(build_ele, handle_prop, input_pnt, doc):
    build_ele.change_property(handle_prop, input_pnt)
    return create_element(build_ele, doc)


class Chamfer:

    def __init__(self, doc):
        self.model_ele_list = []
        self.handle_list = []
        self.document = doc

    def create(self, build_ele):
        self.upps_part(build_ele)
        self.make_handles(build_ele)
        return (self.model_ele_list, self.handle_list)

    def down_part(self, build_ele):
        brepp = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0, 0, 0),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.LengthBot.value,
            build_ele.Witdh.value,
            build_ele.ThickBot.value)

        brepp_2 = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0, 0, 0),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.LengthBot.value,
            build_ele.Witdh.value,
            build_ele.ThickBot.value)

        chamfer_width = build_ele.ChamferWitdh.value
        chamfer_width_bottom = build_ele.ChamferWidth2.value

        if chamfer_width > 0:
            edges = AllplanUtil.VecSizeTList()
            edges.append(1)
            edges.append(3)

            err, brepp = AllplanGeo.ChamferCalculus.Calculate(
                brepp, edges, chamfer_width, False)

            if not GeometryValidate.polyhedron(err):
                return

        if chamfer_width_bottom > 0:
            edges2 = AllplanUtil.VecSizeTList()
            edges2.append(8)
            edges2.append(10)

            err, brepp_2 = AllplanGeo.ChamferCalculus.Calculate(
                brepp_2, edges2, chamfer_width_bottom, False)

            if not GeometryValidate.polyhedron(err):
                return

        err, done_part = AllplanGeo.MakeIntersection(brepp, brepp_2)

        return done_part

    def central_part(self, build_ele):
        brepp = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(build_ele.LengthBot.value / 2 - build_ele.CentralWidth.value / 2, 0, build_ele.ThickBot.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.CentralWidth.value,
            build_ele.Witdh.value,
            build_ele.CentralUp.value)

        cilinder = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(build_ele.ChamferWitdh.value, build_ele.Witdh.value / 8, build_ele.ThickBot.value + build_ele.CentralUp.value / 2),
                                       AllplanGeo.Vector3D(0, 0, 1),
                                       AllplanGeo.Vector3D(1, 0, 0)),
            build_ele.Rad.value , build_ele.CentralWidth.value)

        cilinder_1 = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(build_ele.ChamferWitdh.value, build_ele.Witdh.value - build_ele.Witdh.value / 8, build_ele.ThickBot.value + build_ele.CentralUp.value / 2),
                                       AllplanGeo.Vector3D(0, 0, 1),
                                       AllplanGeo.Vector3D(1, 0, 0)),
            build_ele.Rad.value , build_ele.CentralWidth.value)

        err, brepp = AllplanGeo.MakeSubtraction(brepp, cilinder)
        err, brepp = AllplanGeo.MakeSubtraction(brepp, cilinder_1)

        err, done_part = AllplanGeo.MakeUnion(
            brepp, self.bottom_part(build_ele))
        return done_part

    def top_part(self, build_ele):
        brepp = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0 - (build_ele.Width2.value - build_ele.LengthBot.value) / 2, 0, build_ele.ThickBot.value + build_ele.CentralUp.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.Width2.value,
            build_ele.Witdh.value,
            build_ele.ThickBot2.value)

        brepp_cuboid = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(build_ele.PlSpace.value - (build_ele.Width2.value - build_ele.LengthBot.value) / 2, 0, build_ele.ThickBot.value + build_ele.CentralUp.value + build_ele.ThickBot2.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.Width2.value - build_ele.PlSpace.value*2,
            build_ele.Witdh.value,
            build_ele.PlHeight.value)

        common_p = AllplanBaseElements.CommonProperties()
        common_p.GetGlobalProperties()
        common_p.Pen = 1
        common_p.Color = build_ele.Color_num.value

        chamfer_width_top = build_ele.ChamferWidth2.value

        if chamfer_width_top > 0:
            vecs = AllplanUtil.VecSizeTList()
            vecs.append(8)
            vecs.append(10)

            err, brepp = AllplanGeo.ChamferCalculus.Calculate(
                brepp, vecs, chamfer_width_top, False)

            if not GeometryValidate.polyhedron(err):
                return

        err, done_part = AllplanGeo.MakeUnion(
            brepp, self.central_part(build_ele))
        err, done_part = AllplanGeo.MakeUnion(done_part, brepp_cuboid)
        self.model_ele_list.append(
            AllplanBasisElements.ModelElement3D(common_p, done_part))

    def create_handles(self, build_ele):
        origin = AllplanGeo.Point3D(
            build_ele.LengthBot.value / 2, build_ele.Witdh.value, build_ele.CentralUp.value + build_ele.ThickBot.value)
        origin2 = AllplanGeo.Point3D(
            build_ele.LengthBot.value / 2, 0, build_ele.ThickBot.value / 2)
        origin3 = AllplanGeo.Point3D(
            0, build_ele.Witdh.value, (build_ele.ThickBot.value - build_ele.ChamferWitdh.value) / 2)
        origin4 = AllplanGeo.Point3D(
            0 - (build_ele.Width2.value - build_ele.LengthBot.value) / 2, build_ele.Witdh.value, build_ele.CentralUp.value + build_ele.ThickBot.value + build_ele.ChamferWidth2.value)
        origin5 = AllplanGeo.Point3D(
            build_ele.LengthBot.value / 2, build_ele.Witdh.value, build_ele.CentralUp.value + build_ele.ThickBot.value - build_ele.ThickBot.value / 4)
        origin6 = AllplanGeo.Point3D(
            build_ele.LengthBot.value / 2, build_ele.Witdh.value, build_ele.CentralUp.value + build_ele.ThickBot.value + build_ele.ThickBot2.value)
        origin7 = AllplanGeo.Point3D(
            build_ele.LengthBot.value / 2, build_ele.Witdh.value, 0)
        origin8 = AllplanGeo.Point3D(
            build_ele.LengthBot.value / 2 - build_ele.CentralWidth.value / 2, build_ele.Witdh.value, build_ele.CentralUp.value / 2 + build_ele.ThickBot.value)

        self.handle_list.append(
            HandleProperties("CentralUp",
                             AllplanGeo.Point3D(origin.X,
                                                origin.Y,
                                                origin.Z),
                             AllplanGeo.Point3D(origin.X,
                                                origin.Y,
                                                origin.Z - build_ele.CentralUp.value),
                             [("CentralUp", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handle_list.append(
            HandleProperties("Witdh",
                             AllplanGeo.Point3D(origin2.X,
                                                origin2.Y + build_ele.Witdh.value,
                                                origin2.Z),
                             AllplanGeo.Point3D(origin2.X,
                                                origin2.Y,
                                                origin2.Z),
                             [("Witdh", HandleDirection.y_dir)],
                             HandleDirection.y_dir,
                             False))

        self.handle_list.append(
            HandleProperties("LengthBot", AllplanGeo.Point3D(origin3.X + build_ele.LengthBot.value, origin3.Y, origin3.Z),
                             AllplanGeo.Point3D(
                                 origin3.X, origin3.Y, origin3.Z),
                             [("LengthBot", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))

        self.handle_list.append(
            HandleProperties("Width2",
                             AllplanGeo.Point3D(origin4.X + build_ele.Width2.value,
                                                origin4.Y,
                                                origin4.Z),
                             AllplanGeo.Point3D(origin4.X,
                                                origin4.Y,
                                                origin4.Z),
                             [("Width2", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))

        self.handle_list.append(
            HandleProperties("ThickBot2",
                             AllplanGeo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z + build_ele.ThickBot2.value),
                             AllplanGeo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z),
                             [("ThickBot2", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handle_list.append(
            HandleProperties("PlHeight",
                             AllplanGeo.Point3D(origin6.X,
                                                origin6.Y,
                                                origin6.Z + build_ele.PlHeight.value),
                             AllplanGeo.Point3D(origin6.X,
                                                origin6.Y,
                                                origin6.Z),
                             [("PlHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handle_list.append(
            HandleProperties("ThickBot",
                             AllplanGeo.Point3D(origin7.X,
                                                origin7.Y,
                                                origin7.Z + build_ele.ThickBot.value),
                             AllplanGeo.Point3D(origin7.X,
                                                origin7.Y,
                                                origin7.Z),
                             [("ThickBot", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handle_list.append(
            HandleProperties("CentralWidth",
                             AllplanGeo.Point3D(origin8.X + build_ele.CentralWidth.value,
                                                origin8.Y,
                                                origin8.Z),
                             AllplanGeo.Point3D(origin8.X,
                                                origin8.Y,
                                                origin8.Z),
                             [("CentralWidth", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))
