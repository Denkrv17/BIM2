import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import NemAll_Python_Utility as AllplanUtil
import GeometryValidate as GeometryValidate
from HandleDirection import HandleDirection
from HandleProp import HandleProp


print('Load Chamfer.py')


def check_allplan_version(sozd_element, version):
    del sozd_element
    del version
    return True


def create_element(sozd_element, doc):
    element = Chamfer(doc)
    return element.create(sozd_element)


def move_handle(sozd_element, handle_prop, input_pnt, doc):
    sozd_element.change_property(handle_prop, input_pnt)
    return create_element(sozd_element, doc)


class Chamfer:

    def __init__(self, doc):
        self.model_ele_list = []
        self.handlelist = []
        self.document = doc

    def create(self, sozd_element):
        self.upps_chast(sozd_element)
        self.make_handles(sozd_element)
        return (self.model_ele_list, self.handlelist)

     def verhnya_chast(self, sozd_element):
        brepp3D = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0 - (sozd_element.Width2.value - sozd_element.LengthBot.value) / 2, 0, sozd_element.ThickBot.value + sozd_element.CentralUp.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            sozd_element.Width2.value,
            sozd_element.Witdh.value,
            sozd_element.ThickBot2.value)

        brepp3D_cuboid = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(sozd_element.PlSpace.value - (sozd_element.Width2.value - sozd_element.LengthBot.value) / 2, 0, sozd_element.ThickBot.value + sozd_element.CentralUp.value + sozd_element.ThickBot2.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            sozd_element.Width2.value - sozd_element.PlSpace.value*2,
            sozd_element.Witdh.value,
            sozd_element.PlHeight.value)

        commad_propertis = AllplanBaseElements.CommonProperties()
        commad_propertis.GetGlobalProperties()
        commad_propertis.Pen = 1
        commad_propertis.Color = sozd_element.Color_num.value

        chamwid_verhnya = sozd_element.ChamferWidth2.value

        if chamwid_verhnya > 0:
            vecs = AllplanUtil.VecSizeTList()
            vecs.append(8)
            vecs.append(10)

            pomylka, brepp3D = AllplanGeo.ChamferCalculus.Calculate(
                brepp3D, vecs, chamwid_verhnya, False)

            if not GeometryValidate.polyhedron(pomylka):
                return

        pomylka, done_chast = AllplanGeo.MakeUnion(
            brepp3D, self.central_chast(sozd_element))
        pomylka, done_chast = AllplanGeo.MakeUnion(done_chast, brepp3D_cuboid)
        self.model_ele_list.append(
            AllplanBasisElements.ModelElement3D(commad_propertis, done_chast))

    

    def central_chast(self, sozd_element):
        brepp3D = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(sozd_element.LengthBot.value / 2 - sozd_element.CentralWidth.value / 2, 0, sozd_element.ThickBot.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            sozd_element.CentralWidth.value,
            sozd_element.Witdh.value,
            sozd_element.CentralUp.value)

        cilin = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(sozd_element.ChamferWitdh.value, sozd_element.Witdh.value / 8, sozd_element.ThickBot.value + sozd_element.CentralUp.value / 2),
                                       AllplanGeo.Vector3D(0, 0, 1),
                                       AllplanGeo.Vector3D(1, 0, 0)),
            sozd_element.Rad.value , sozd_element.CentralWidth.value)

        cilin_1 = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(sozd_element.ChamferWitdh.value, sozd_element.Witdh.value - sozd_element.Witdh.value / 8, sozd_element.ThickBot.value + sozd_element.CentralUp.value / 2),
                                       AllplanGeo.Vector3D(0, 0, 1),
                                       AllplanGeo.Vector3D(1, 0, 0)),
            sozd_element.Rad.value , sozd_element.CentralWidth.value)

        pomylka, brepp3D = AllplanGeo.MakeSubtraction(brepp3D, cilin)
        pomylka, brepp3D = AllplanGeo.MakeSubtraction(brepp3D, cilin_1)

        pomylka, done_chast = AllplanGeo.MakeUnion(
            brepp3D, self.bottom_chast(sozd_element))
        return done_chast

    def nizhnya_chast(self, sozd_element):
        brepp3D = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0, 0, 0),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            sozd_element.LengthBot.value,
            sozd_element.Witdh.value,
            sozd_element.ThickBot.value)

        brepp3D_2 = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0, 0, 0),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            sozd_element.LengthBot.value,
            sozd_element.Witdh.value,
            sozd_element.ThickBot.value)

        chamwid = sozd_element.ChamferWitdh.value
        chamwid_bottom = sozd_element.ChamferWidth2.value

        if chamwid > 0:
            edVecsize = AllplanUtil.VecSizeTList()
            edVecsize.append(1)
            edVecsize.append(3)

            pomylka, brepp3D = AllplanGeo.ChamferCalculus.Calculate(
                brepp3D, edVecsize, chamwid, False)

            if not GeometryValidate.polyhedron(pomylka):
                return

        if chamwid_bottom > 0:
            edVecsize2 = AllplanUtil.VecSizeTList()
            edVecsize2.append(8)
            edVecsize2.append(10)

            pomylka, brepp3D_2 = AllplanGeo.ChamferCalculus.Calculate(
                brepp3D_2, edVecsize2, chamwid_bottom, False)

            if not GeometryValidate.polyhedron(pomylka):
                return

        pomylka, done_chast = AllplanGeo.MakeIntersection(brepp3D, brepp3D_2)

        return done_chast
   

    def create_handles(self, sozd_element):
        origin = AllplanGeo.Point3D(
            sozd_element.LengthBot.value / 2, sozd_element.Witdh.value, sozd_element.CentralUp.value + sozd_element.ThickBot.value)
        origin2 = AllplanGeo.Point3D(
            sozd_element.LengthBot.value / 2, 0, sozd_element.ThickBot.value / 2)
        origin3 = AllplanGeo.Point3D(
            0, sozd_element.Witdh.value, (sozd_element.ThickBot.value - sozd_element.ChamferWitdh.value) / 2)
        origin4 = AllplanGeo.Point3D(
            0 - (sozd_element.Width2.value - sozd_element.LengthBot.value) / 2, sozd_element.Witdh.value, sozd_element.CentralUp.value + sozd_element.ThickBot.value + sozd_element.ChamferWidth2.value)
        origin5 = AllplanGeo.Point3D(
            sozd_element.LengthBot.value / 2, sozd_element.Witdh.value, sozd_element.CentralUp.value + sozd_element.ThickBot.value - sozd_element.ThickBot.value / 4)
        origin6 = AllplanGeo.Point3D(
            sozd_element.LengthBot.value / 2, sozd_element.Witdh.value, sozd_element.CentralUp.value + sozd_element.ThickBot.value + sozd_element.ThickBot2.value)
        origin7 = AllplanGeo.Point3D(
            sozd_element.LengthBot.value / 2, sozd_element.Witdh.value, 0)
        origin8 = AllplanGeo.Point3D(
            sozd_element.LengthBot.value / 2 - sozd_element.CentralWidth.value / 2, sozd_element.Witdh.value, sozd_element.CentralUp.value / 2 + sozd_element.ThickBot.value)

        self.handlelist.append(
            HandleProp("CentralUp",
                             AllplanGeo.Point3D(origin.X,
                                                origin.Y,
                                                origin.Z),
                             AllplanGeo.Point3D(origin.X,
                                                origin.Y,
                                                origin.Z - sozd_element.CentralUp.value),
                             [("CentralUp", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handlelist.append(
            HandleProp("Witdh",
                             AllplanGeo.Point3D(origin2.X,
                                                origin2.Y + sozd_element.Witdh.value,
                                                origin2.Z),
                             AllplanGeo.Point3D(origin2.X,
                                                origin2.Y,
                                                origin2.Z),
                             [("Witdh", HandleDirection.y_dir)],
                             HandleDirection.y_dir,
                             False))

        self.handlelist.append(
            HandleProp("LengthBot", AllplanGeo.Point3D(origin3.X + sozd_element.LengthBot.value, origin3.Y, origin3.Z),
                             AllplanGeo.Point3D(
                                 origin3.X, origin3.Y, origin3.Z),
                             [("LengthBot", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))

        self.handlelist.append(
            HandleProp("Width2",
                             AllplanGeo.Point3D(origin4.X + sozd_element.Width2.value,
                                                origin4.Y,
                                                origin4.Z),
                             AllplanGeo.Point3D(origin4.X,
                                                origin4.Y,
                                                origin4.Z),
                             [("Width2", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))

        self.handlelist.append(
            HandleProp("ThickBot2",
                             AllplanGeo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z + sozd_element.ThickBot2.value),
                             AllplanGeo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z),
                             [("ThickBot2", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handlelist.append(
            HandleProp("PlHeight",
                             AllplanGeo.Point3D(origin6.X,
                                                origin6.Y,
                                                origin6.Z + sozd_element.PlHeight.value),
                             AllplanGeo.Point3D(origin6.X,
                                                origin6.Y,
                                                origin6.Z),
                             [("PlHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handlelist.append(
            HandleProp("ThickBot",
                             AllplanGeo.Point3D(origin7.X,
                                                origin7.Y,
                                                origin7.Z + sozd_element.ThickBot.value),
                             AllplanGeo.Point3D(origin7.X,
                                                origin7.Y,
                                                origin7.Z),
                             [("ThickBot", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handlelist.append(
            HandleProp("CentralWidth",
                             AllplanGeo.Point3D(origin8.X + sozd_element.CentralWidth.value,
                                                origin8.Y,
                                                origin8.Z),
                             AllplanGeo.Point3D(origin8.X,
                                                origin8.Y,
                                                origin8.Z),
                             [("CentralWidth", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))
