# coding=utf-8
from __future__ import print_function

import math
import os
from datetime import datetime
import rospkg
from PyQt5 import uic

import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QHeaderView, QAction, QApplication, QFileDialog
from rqt_gui_py.plugin import Plugin
from sensor_calib_msgs.msg import Residuals, UseResiduals, UseResidual, EstimatedPoses, OptimizeSensorPoses, \
    OptimizeSensorPose, Reset, DetectionStati, DetectionStatus, MeasurementIndex, MeasurementIndices
from std_msgs.msg import String, Empty, Bool
from scipy.spatial.transform import Rotation


class Dispatcher(QtCore.QObject):
    residuals_msg = QtCore.pyqtSignal(object)
    overall_residuals_msg = QtCore.pyqtSignal(object)
    board_poses_msg = QtCore.pyqtSignal(object)
    sensor_poses_msg = QtCore.pyqtSignal(object)
    detection_status_msg = QtCore.pyqtSignal(object)
    measurement_indices_msg = QtCore.pyqtSignal(object)


class MyTableModel(QtCore.QAbstractTableModel):
    def __init__(self, parent=None, *args):
        super(MyTableModel, self).__init__()
        self.header_labels = []

    def columnCount(self, parent=QtCore.QModelIndex()):
        return len(self.header_labels)

    def headerData(self, section, orientation, role):
        if role == QtCore.Qt.DisplayRole:
            if orientation == QtCore.Qt.Horizontal:
                return QtCore.QVariant(self.header_labels[section])

        return QtCore.QAbstractTableModel.headerData(self, section, orientation, role)

    def flags(self, index):
        return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled


class ResidualsTableModel(MyTableModel):
    def __init__(self, parent=None, *args):
        super(ResidualsTableModel, self).__init__()
        self.residual_groups = []
        self.header_labels = ["Sensor", "Board", "Unit", "RMSE", "Min.", "Max.", "Count"]

    def update(self, msg):
        if QApplication.mouseButtons() == QtCore.Qt.NoButton:
            self.beginResetModel()
            self.residual_groups = msg.residual_groups
            self.endResetModel()

            self.layoutChanged.emit()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.residual_groups)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        col = index.column()
        row = index.row()

        residual_group = self.residual_groups[row]

        if role == QtCore.Qt.DisplayRole:
            if col == 0:
                return residual_group.group
            elif col == 1:
                return residual_group.name
            elif col == 2:
                return residual_group.unit
            elif col == 3:
                return "%.3f" % residual_group.rmse
            elif col == 4:
                return "%.3f" % residual_group.min_residual
            elif col == 5:
                return "%.3f" % residual_group.max_residual
            elif col == 6:
                return str(residual_group.count)

        #  Fallback
        return QtCore.QVariant()


class ResidualsTableModelWithCheckbox(ResidualsTableModel):
    def __init__(self, parent=None, *args):
        super(ResidualsTableModelWithCheckbox, self).__init__()
        self.header_labels.append("Use")
        self.use = {}
        self.use_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/measurements_to_use',
                                       UseResiduals,
                                       queue_size=1)

    def setData(self, index, value, role):
        if not index.isValid():
            return False

        if role == QtCore.Qt.CheckStateRole:
            row = index.row()
            residual_group = self.residual_groups[row]

            self.beginResetModel()
            self.use[(residual_group.group, residual_group.name)] = value == QtCore.Qt.Checked
            self.endResetModel()

            self.layoutChanged.emit()

            msg = UseResiduals()
            for r in self.residual_groups:
                r_msg = UseResidual()
                r_msg.group = r.group
                r_msg.name = r.name

                if (r.group, r.name) not in self.use:
                    self.use[(r.group, r.name)] = True

                r_msg.use = self.use[(r.group, r.name)]
                msg.use_residuals.append(r_msg)
            self.use_pub.publish(msg)

        return ResidualsTableModel.setData(self, index, value, role)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.CheckStateRole:
            col = index.column()
            row = index.row()
            if col == 7:
                residual_group = self.residual_groups[row]
                if not (residual_group.group, residual_group.name) in self.use:
                    self.use[(residual_group.group, residual_group.name)] = True
                if self.use[(residual_group.group, residual_group.name)]:
                    return QtCore.Qt.Checked
                else:
                    return QtCore.Qt.Unchecked

        #  Fallback
        return ResidualsTableModel.data(self, index, role)

    def flags(self, index):
        col = index.column()
        if col == 7:
            return QtCore.Qt.ItemIsUserCheckable | ResidualsTableModel.flags(self, index)
        else:
            return ResidualsTableModel.flags(self, index)


class TwoCheckDelegate(QtWidgets.QStyledItemDelegate):
    def __init__(self, parent=None):
        super().__init__(parent)

    def createEditor(self, parent, option, index):
        w = QtWidgets.QWidget(parent)
        rot = QtWidgets.QCheckBox("T", w)
        trans = QtWidgets.QCheckBox("R", w)

        rot.toggled.connect(lambda _c: self.commitData.emit(w))
        trans.toggled.connect(lambda _c: self.commitData.emit(w))

        lay = QtWidgets.QHBoxLayout(w)
        lay.setContentsMargins(4, 0, 4, 0)
        lay.setSpacing(6)
        lay.addWidget(rot)
        lay.addWidget(trans)
        return w

    def setEditorData(self, editor, index):
        mask = int(index.model().data(index, QtCore.Qt.EditRole) or 0)
        for i in range(editor.layout().count()):
            cb = editor.layout().itemAt(i).widget()
            blocker = QtCore.QSignalBlocker(cb)
            if i == 0:  # Rot (Bit 0)
                cb.setChecked(bool(mask & 1))
            elif i == 1:  # Trans (Bit 1)
                cb.setChecked(bool(mask & 2))

    def setModelData(self, editor, model, index):
        mask = 0
        for i in range(editor.layout().count()):
            cb = editor.layout().itemAt(i).widget()
            if i == 0 and cb.isChecked():
                mask |= 1
            elif i == 1 and cb.isChecked():
                mask |= 2
        model.setData(index, mask, QtCore.Qt.EditRole)

    def updateEditorGeometry(self, editor, option, index):
        editor.setGeometry(option.rect)


class SensorPoseTableModel(MyTableModel):
    def __init__(self, parent=None, *args):
        super(SensorPoseTableModel, self).__init__()
        self.poses = []
        self.header_labels = [
            "from", "to", "Translation (in m)", "Euler rotation (in deg.)", "Quat. rotation", "Distance (in m)",
            "Optimize"
        ]
        self.optimize_bits = {}
        self.optimize_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/sensor_poses_to_optimize',
                                            OptimizeSensorPoses,
                                            queue_size=1)

    def update(self, msg):
        if QApplication.mouseButtons() == QtCore.Qt.NoButton:
            self.beginResetModel()
            self.poses = msg.poses
            self.endResetModel()

            self.layoutChanged.emit()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.poses)

    def ensure_default_mask(self, mask):
        return mask if mask is not None else (1 | 2)

    def setData(self, index, value, role):
        if not index.isValid():
            return False

        if role == QtCore.Qt.EditRole and index.column() == 6:
            row = index.row()
            pose = self.poses[row]

            mask = int(value)

            self.optimize_bits[(pose.header.frame_id, pose.child_frame_id)] = mask

            msg = OptimizeSensorPoses()
            for p in self.poses:
                p_msg = OptimizeSensorPose()
                p_msg.from_frame = p.header.frame_id
                p_msg.to_frame = p.child_frame_id

                if (p.header.frame_id, p.child_frame_id) not in self.optimize_bits:
                    self.optimize_bits[(p.header.frame_id, p.child_frame_id)] = (1 | 2)

                p_msg.optimize_translation = bool(self.optimize_bits[(p.header.frame_id, p.child_frame_id)] & 1)
                p_msg.optimize_rotation = bool(self.optimize_bits[(p.header.frame_id, p.child_frame_id)] & 2)
                msg.optimize_sensor_poses.append(p_msg)
            self.optimize_pub.publish(msg)

        return MyTableModel.setData(self, index, value, role)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        col = index.column()
        row = index.row()

        pose = self.poses[row]

        if role == QtCore.Qt.DisplayRole:
            t = pose.transform.translation
            q = pose.transform.rotation

            euler = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)

            if col == 0:
                return pose.header.frame_id
            elif col == 1:
                return pose.child_frame_id
            elif col == 2:
                return "x: %.3f, y: %.3f, z: %.3f" % (t.x, t.y, t.z)
            elif col == 3:
                return "r: %.3f, p: %.3f, y: %.3f" % (euler[0], euler[1], euler[2])
            elif col == 4:
                return "x: %.3f, y: %.3f, z: %.3f, w: %.3f" % (q.x, q.y, q.z, q.w)
            elif col == 5:
                return "%.3f" % math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)

        if role == QtCore.Qt.EditRole and col == 6:
            mask = self.ensure_default_mask(self.optimize_bits.get((pose.header.frame_id, pose.child_frame_id)))
            return mask

        #  Fallback
        return QtCore.QVariant()

    def flags(self, index):
        col = index.column()
        if col == 6:
            return QtCore.Qt.ItemIsEditable | MyTableModel.flags(self, index)
        else:
            return MyTableModel.flags(self, index)


class BoardPoseTableModel(MyTableModel):
    def __init__(self, parent=None, *args):
        super(BoardPoseTableModel, self).__init__()
        self.poses = []
        self.header_labels = [
            "from", "to", "Translation (in m)", "Euler rotation (in deg.)", "Quat. rotation", "Distance (in m)",
            "Optimize"
        ]
        self.optimize = {}
        self.optimize_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/sensor_poses_to_optimize',
                                            OptimizeSensorPoses,
                                            queue_size=1)

    def update(self, msg):
        if QApplication.mouseButtons() == QtCore.Qt.NoButton:
            self.beginResetModel()
            self.poses = msg.poses
            self.endResetModel()

            self.layoutChanged.emit()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.poses)

    def setData(self, index, value, role):
        if not index.isValid():
            return False

        if role == QtCore.Qt.CheckStateRole:
            row = index.row()
            pose = self.poses[row]

            self.beginResetModel()
            self.optimize[(pose.header.frame_id, pose.child_frame_id)] = value == QtCore.Qt.Checked
            self.endResetModel()

            self.layoutChanged.emit()

            msg = OptimizeSensorPoses()
            for p in self.poses:
                p_msg = OptimizeSensorPose()
                p_msg.from_frame = p.header.frame_id
                p_msg.to_frame = p.child_frame_id

                if (p.header.frame_id, p.child_frame_id) not in self.optimize:
                    self.optimize[(p.header.frame_id, p.child_frame_id)] = True

                p_msg.optimize = self.optimize[(p.header.frame_id, p.child_frame_id)]
                msg.optimize_sensor_poses.append(p_msg)
            self.optimize_pub.publish(msg)

        return MyTableModel.setData(self, index, value, role)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        col = index.column()
        row = index.row()

        if role == QtCore.Qt.DisplayRole:
            pose = self.poses[row]
            t = pose.transform.translation
            q = pose.transform.rotation

            euler = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)

            if col == 0:
                return pose.header.frame_id
            elif col == 1:
                return pose.child_frame_id
            elif col == 2:
                return "x: %.3f, y: %.3f, z: %.3f" % (t.x, t.y, t.z)
            elif col == 3:
                return "r: %.3f, p: %.3f, y: %.3f" % (euler[0], euler[1], euler[2])
            elif col == 4:
                return "x: %.3f, y: %.3f, z: %.3f, w: %.3f" % (q.x, q.y, q.z, q.w)
            elif col == 5:
                return "%.3f" % math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)

        if role == QtCore.Qt.CheckStateRole:
            if col == 6:
                pose = self.poses[row]
                if not (pose.header.frame_id, pose.child_frame_id) in self.optimize:
                    self.optimize[(pose.header.frame_id, pose.child_frame_id)] = True
                if self.optimize[(pose.header.frame_id, pose.child_frame_id)]:
                    return QtCore.Qt.Checked
                else:
                    return QtCore.Qt.Unchecked

        #  Fallback
        return QtCore.QVariant()

    def flags(self, index):
        col = index.column()
        if col == 6:
            return QtCore.Qt.ItemIsUserCheckable | MyTableModel.flags(self, index)
        else:
            return MyTableModel.flags(self, index)


class DetectionStatusTableModel(MyTableModel):
    def __init__(self, parent=None, *args):
        super(DetectionStatusTableModel, self).__init__()
        self.detection_status = []
        self.header_labels = [
            "Sensor", "Num. Detections", "Age (in s)"
        ]

    def update(self, msg):
        if QApplication.mouseButtons() == QtCore.Qt.NoButton:
            self.beginResetModel()
            self.detection_status = msg.detection_stati
            self.endResetModel()

            self.layoutChanged.emit()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.detection_status)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        col = index.column()
        row = index.row()

        detection_status = self.detection_status[row]

        if role == QtCore.Qt.DisplayRole:
            if col == 0:
                return detection_status.sensor
            elif col == 1:
                return detection_status.num_detections
            elif col == 2:
                return "%.2f" % (rospy.Time.now().to_sec() - detection_status.stamp.to_sec())

        if role == QtCore.Qt.BackgroundRole:
            if col == 1:
                if detection_status.num_detections > 0:
                    return QtGui.QColor('#00FF00')
                else:
                    return QtGui.QColor('#FF0000')
            elif col == 2:
                if (rospy.Time.now().to_sec() - detection_status.stamp.to_sec()) < 2:
                    return QtGui.QColor('#00FF00')
                else:
                    return QtGui.QColor('#FF0000')

        #  Fallback
        return QtCore.QVariant()


class ExtrinsicSensorCalib(Plugin):
    slider_factor = 1000.0

    def __init__(self, context):
        super(ExtrinsicSensorCalib, self).__init__(context)
        self.setObjectName('ExtrinsicSensorCalib')

        self._disp = Dispatcher()

        self._widget = QWidget()
        self.rospgk = rospkg.RosPack()
        ui_file = os.path.join(self.rospgk.get_path('rqt_sensor_calib'), 'resource', 'ExtrinsicSensorCalib.ui')
        uic.loadUi(ui_file, self._widget)
        self._widget.setObjectName('ExtrinsicSensorCalibUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.residuals_model = ResidualsTableModelWithCheckbox()
        residuals_sorter_model = QtCore.QSortFilterProxyModel()
        residuals_sorter_model.setSourceModel(self.residuals_model)
        residuals_sorter_model.setSortRole(QtCore.Qt.EditRole)
        self._widget.residuals_table_view.setModel(residuals_sorter_model)
        self._widget.residuals_table_view.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents)  # Sensor
        self._widget.residuals_table_view.horizontalHeader().setSectionResizeMode(1,
                                                                                  QHeaderView.ResizeToContents)  # Board
        self._widget.residuals_table_view.horizontalHeader().setSectionResizeMode(2,
                                                                                  QHeaderView.ResizeToContents)  # Unit
        self._widget.residuals_table_view.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)  # RMSE
        self._widget.residuals_table_view.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)  # Min.
        self._widget.residuals_table_view.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)  # Max.
        self._widget.residuals_table_view.horizontalHeader().setSectionResizeMode(6, QHeaderView.Stretch)  # Count

        self.overall_residuals_model = ResidualsTableModel()
        overall_residuals_sorter_model = QtCore.QSortFilterProxyModel()
        overall_residuals_sorter_model.setSourceModel(self.overall_residuals_model)
        overall_residuals_sorter_model.setSortRole(QtCore.Qt.EditRole)
        self._widget.overall_residuals_table_view.setModel(overall_residuals_sorter_model)
        self._widget.overall_residuals_table_view.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents)  # Sensor
        self._widget.overall_residuals_table_view.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeToContents)  # Board
        self._widget.overall_residuals_table_view.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeToContents)  # Unit
        self._widget.overall_residuals_table_view.horizontalHeader().setSectionResizeMode(3,
                                                                                          QHeaderView.Stretch)  # RMSE
        self._widget.overall_residuals_table_view.horizontalHeader().setSectionResizeMode(4,
                                                                                          QHeaderView.Stretch)  # Min.
        self._widget.overall_residuals_table_view.horizontalHeader().setSectionResizeMode(5,
                                                                                          QHeaderView.Stretch)  # Max.
        self._widget.overall_residuals_table_view.horizontalHeader().setSectionResizeMode(6,
                                                                                          QHeaderView.Stretch)  # Count

        self.sensor_poses_model = SensorPoseTableModel()
        self._widget.sensor_poses_table_view.setModel(self.sensor_poses_model)
        self._widget.sensor_poses_table_view.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents)  # from
        self._widget.sensor_poses_table_view.horizontalHeader().setSectionResizeMode(1,
                                                                                     QHeaderView.ResizeToContents)  # to
        self._widget.sensor_poses_table_view.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeToContents)  # translation
        self._widget.sensor_poses_table_view.horizontalHeader().setSectionResizeMode(
            3, QHeaderView.ResizeToContents)  # rotation
        self._widget.sensor_poses_table_view.horizontalHeader().setSectionResizeMode(4,
                                                                                     QHeaderView.Stretch)  # quaternion
        self._widget.sensor_poses_table_view.horizontalHeader().setSectionResizeMode(5,
                                                                                     QHeaderView.ResizeToContents)  # distance
        self._widget.sensor_poses_table_view.horizontalHeader().setSectionResizeMode(6,
                                                                                     QHeaderView.ResizeToContents)  # optimize

        delegate = TwoCheckDelegate(self._widget.sensor_poses_table_view)
        self._widget.sensor_poses_table_view.setItemDelegateForColumn(6, delegate)
        for r in range(self.sensor_poses_model.rowCount()):
            self._widget.sensor_poses_table_view.openPersistentEditor(self.sensor_poses_model.index(r, 6))

        def _open_all():
            for r in range(self.sensor_poses_model.rowCount()):
                self._widget.sensor_poses_table_view.openPersistentEditor(self.sensor_poses_model.index(r, 6))

        self.sensor_poses_model.rowsInserted.connect(lambda _p, _f, _l: _open_all())
        self.sensor_poses_model.modelReset.connect(_open_all)
        self.sensor_poses_model.layoutChanged.connect(_open_all)

        self.board_poses_model = BoardPoseTableModel()
        self._widget.board_poses_table_view.setModel(self.board_poses_model)
        self._widget.board_poses_table_view.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents)  # from
        self._widget.board_poses_table_view.horizontalHeader().setSectionResizeMode(1,
                                                                                    QHeaderView.ResizeToContents)  # to
        self._widget.board_poses_table_view.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeToContents)  # translation
        self._widget.board_poses_table_view.horizontalHeader().setSectionResizeMode(
            3, QHeaderView.ResizeToContents)  # rotation
        self._widget.board_poses_table_view.horizontalHeader().setSectionResizeMode(4,
                                                                                    QHeaderView.Stretch)  # quaternion
        self._widget.board_poses_table_view.horizontalHeader().setSectionResizeMode(5,
                                                                                    QHeaderView.ResizeToContents)  # distance
        self._widget.board_poses_table_view.horizontalHeader().setSectionResizeMode(6,
                                                                                    QHeaderView.ResizeToContents)  # optimize

        self._widget.sensor_poses_table_view.setContextMenuPolicy(QtCore.Qt.ActionsContextMenu)
        copy_sensor_pose_action = QAction("Copy sensor pose for yaml file", self)
        copy_sensor_pose_action.triggered.connect(self.copySensorPoseToClipboard)
        self._widget.sensor_poses_table_view.addAction(copy_sensor_pose_action)
        reset_sensor_pose_initial_tf_action = QAction("Reset sensor pose from initial tf", self)
        reset_sensor_pose_initial_tf_action.triggered.connect(self.resetSensorPoseFromInitialTF)
        self._widget.sensor_poses_table_view.addAction(reset_sensor_pose_initial_tf_action)
        reset_sensor_pose_current_tf_action = QAction("Reset sensor pose from current tf", self)
        reset_sensor_pose_current_tf_action.triggered.connect(self.resetSensorPoseFromCurrentTF)
        self._widget.sensor_poses_table_view.addAction(reset_sensor_pose_current_tf_action)
        reset_sensor_pose_zero_action = QAction("Reset sensor pose to zero", self)
        reset_sensor_pose_zero_action.triggered.connect(self.resetSensorPoseToZero)
        self._widget.sensor_poses_table_view.addAction(reset_sensor_pose_zero_action)
        self._widget.board_poses_table_view.setContextMenuPolicy(QtCore.Qt.ActionsContextMenu)
        reset_board_pose_action = QAction("Reset board pose", self)
        reset_board_pose_action.triggered.connect(self.resetBoardPose)
        self._widget.board_poses_table_view.addAction(reset_board_pose_action)

        self.detection_status_model = DetectionStatusTableModel()
        self._widget.detection_status_table_view.setModel(self.detection_status_model)
        self._widget.detection_status_table_view.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents)  # Sensor
        self._widget.detection_status_table_view.horizontalHeader().setSectionResizeMode(1,
                                                                                         QHeaderView.ResizeToContents)  # Num
        self._widget.detection_status_table_view.horizontalHeader().setSectionResizeMode(2,
                                                                                         QHeaderView.ResizeToContents)  # Age

        self._widget.measurement_idx_combo_box.currentIndexChanged.connect(self._onMeasurementComboChanged)
        self.last_published_measurement = None

        self._disp.residuals_msg.connect(self.residuals_model.update, QtCore.Qt.QueuedConnection)
        self._disp.overall_residuals_msg.connect(self.overall_residuals_model.update, QtCore.Qt.QueuedConnection)
        self._disp.board_poses_msg.connect(self.board_poses_model.update, QtCore.Qt.QueuedConnection)
        self._disp.sensor_poses_msg.connect(self.sensor_poses_model.update, QtCore.Qt.QueuedConnection)
        self._disp.detection_status_msg.connect(self.detection_status_model.update, QtCore.Qt.QueuedConnection)
        self._disp.measurement_indices_msg.connect(self._updateMeasurementCombo, QtCore.Qt.QueuedConnection)

        self.subs = []
        self.subs.append(rospy.Subscriber("/sensor_calib/extrinsic_sensor_calib/residuals",
                                          Residuals, lambda m: self._disp.residuals_msg.emit(m)))
        self.subs.append(rospy.Subscriber("/sensor_calib/extrinsic_sensor_calib/overall_residuals",
                                          Residuals, lambda m: self._disp.overall_residuals_msg.emit(m)))
        self.subs.append(rospy.Subscriber("/sensor_calib/extrinsic_sensor_calib/status",
                                          String, self._statusCallback))
        self.subs.append(rospy.Subscriber("/sensor_calib/extrinsic_sensor_calib/board_poses",
                                          EstimatedPoses, lambda m: self._disp.board_poses_msg.emit(m)))
        self.subs.append(rospy.Subscriber("/sensor_calib/extrinsic_sensor_calib/sensor_poses",
                                          EstimatedPoses, lambda m: self._disp.sensor_poses_msg.emit(m)))
        self.subs.append(rospy.Subscriber("/sensor_calib/extrinsic_sensor_calib/detection_status",
                                          DetectionStati, lambda m: self._disp.detection_status_msg.emit(m)))
        self.subs.append(rospy.Subscriber("/sensor_calib/extrinsic_sensor_calib/measurement_indices",
                                          MeasurementIndices, lambda m: self._disp.measurement_indices_msg.emit(m)))

        self.measure_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/measure', Bool, queue_size=1)
        self.remove_last_measurement_pub = rospy.Publisher(
            '/sensor_calib/extrinsic_sensor_calib/requests/remove_last_measurement', Empty, queue_size=1)

        self.solve_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/solve', Empty, queue_size=1)
        self.verify_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/verify', Empty, queue_size=1)

        self.reset_sensor_pose_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/reset_sensor_pose',
                                                     Reset,
                                                     queue_size=1)
        self.reset_board_pose_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/reset_board_pose',
                                                    Reset,
                                                    queue_size=1)

        self.preview_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/preview', Bool, queue_size=1)

        self.save_to_file_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/save_to_file',
                                                String,
                                                queue_size=1)
        self.load_from_file_pub = rospy.Publisher('/sensor_calib/extrinsic_sensor_calib/requests/load_from_file',
                                                  String,
                                                  queue_size=1)

        self.measurement_idx_pub = rospy.Publisher(
            '/sensor_calib/extrinsic_sensor_calib/requests/measurement_idx_to_visualize', MeasurementIndex,
            queue_size=1)

        self._widget.measure_button.clicked.connect(self._measureButtonClicked)
        self._widget.amend_button.clicked.connect(self._amendButtonClicked)
        self._widget.remove_last_measurement_button.clicked.connect(self._removeLastMeasurementButtonClicked)

        self._widget.solve_button.clicked.connect(self._solveButtonClicked)
        self._widget.verify_button.clicked.connect(self._verifyButtonClicked)

        self._widget.reset_button.clicked.connect(self._resetButtonClicked)
        self._widget.preview_button.toggled.connect(self._previewButtonToggled)

        self._widget.save_to_file_button.clicked.connect(self._saveToFileButtonClicked)
        self._widget.load_from_file_button.clicked.connect(self._loadFromFileButtonClicked)

        self.file_dialog = QFileDialog(directory=QtCore.QDir.homePath())

        self.last_status_update_time = datetime.min

        self.timeout_timer = QtCore.QTimer(self)
        self.timeout_timer.timeout.connect(self.timeoutCallback)
        self.timeout_timer.start(100)

    def shutdown_plugin(self):
        try:
            self.timeout_timer.stop()
        except Exception:
            pass
        try:
            for s in getattr(self, "subs", []):
                try:
                    s.unregister()
                except Exception:
                    pass
        except Exception:
            pass
        for pub in ["measure_pub", "remove_last_measurement_pub", "solve_pub", "verify_pub",
                    "reset_sensor_pose_pub", "reset_board_pose_pub", "preview_pub",
                    "save_to_file_pub", "load_from_file_pub", "measurement_idx_pub"]:
            p = getattr(self, pub, None)
            try:
                p.unregister()
            except Exception:
                pass

    def timeoutCallback(self):
        age = datetime.now() - self.last_status_update_time
        if age.total_seconds() > 2.0:
            self._widget.status_label.setText("No connection to extrinsic_sensor_calib node!")

    def copySensorPoseToClipboard(self):
        selected_indices = self._widget.sensor_poses_table_view.selectionModel().selectedRows()
        if len(selected_indices) > 0:
            pose = self.sensor_poses_model.poses[selected_indices[0].row()]
            t = pose.transform.translation
            q = pose.transform.rotation

            pose_str = "pose: {{x: {0:.6f}, y: {1:.6f}, z: {2:.6f}, q_x: {3:.16f}, q_y: {4:.16f}, q_z: {5:.16f}, q_w: {6:.16f}}}".format(
                t.x, t.y, t.z, q.x, q.y, q.z, q.w)
            clipboard = QApplication.clipboard()
            clipboard.setText(pose_str)

    def resetSensorPoseFromInitialTF(self):
        selected_indices = self._widget.sensor_poses_table_view.selectionModel().selectedRows()
        if len(selected_indices) > 0:
            pose = self.sensor_poses_model.poses[selected_indices[0].row()]
            msg = Reset()
            msg.from_frame_id = pose.header.frame_id
            msg.to_frame_id = pose.child_frame_id
            msg.zero = False
            msg.current_tf = False
            self.reset_sensor_pose_pub.publish(msg)

    def resetSensorPoseFromCurrentTF(self):
        selected_indices = self._widget.sensor_poses_table_view.selectionModel().selectedRows()
        if len(selected_indices) > 0:
            pose = self.sensor_poses_model.poses[selected_indices[0].row()]
            msg = Reset()
            msg.from_frame_id = pose.header.frame_id
            msg.to_frame_id = pose.child_frame_id
            msg.zero = False
            msg.current_tf = True
            self.reset_sensor_pose_pub.publish(msg)

    def resetSensorPoseToZero(self):
        selected_indices = self._widget.sensor_poses_table_view.selectionModel().selectedRows()
        if len(selected_indices) > 0:
            pose = self.sensor_poses_model.poses[selected_indices[0].row()]
            msg = Reset()
            msg.from_frame_id = pose.header.frame_id
            msg.to_frame_id = pose.child_frame_id
            msg.zero = True
            msg.current_tf = False
            self.reset_sensor_pose_pub.publish(msg)

    def resetBoardPose(self):
        selected_indices = self._widget.board_poses_table_view.selectionModel().selectedRows()
        if len(selected_indices) > 0:
            pose = self.board_poses_model.poses[selected_indices[0].row()]
            msg = Reset()
            msg.from_frame_id = pose.header.frame_id
            msg.to_frame_id = pose.child_frame_id
            msg.zero = False
            self.reset_board_pose_pub.publish(msg)

    def _measureButtonClicked(self):
        msg = Bool()
        msg.data = True
        self.measure_pub.publish(msg)

    def _amendButtonClicked(self):
        msg = Bool()
        msg.data = False
        self.measure_pub.publish(msg)

    def _removeLastMeasurementButtonClicked(self):
        self.remove_last_measurement_pub.publish(Empty())

    def _solveButtonClicked(self):
        self.solve_pub.publish(Empty())

    def _verifyButtonClicked(self):
        self.verify_pub.publish(Empty())

    def _resetButtonClicked(self):
        msg = Reset()
        msg.from_frame_id = ''
        msg.to_frame_id = ''
        msg.zero = False
        self.reset_sensor_pose_pub.publish(msg)
        self.reset_board_pose_pub.publish(msg)

    def _previewButtonToggled(self, checked):
        msg = Bool()
        msg.data = checked
        self.preview_pub.publish(msg)

    def _saveToFileButtonClicked(self):
        filename = self.file_dialog.getSaveFileName(self._widget, caption="Save to file",
                                                    filter="Binary Files (*.bin);;All Files (*)")
        filename = filename[0]
        if not filename.endswith('.bin'):
            filename += ".bin"

        self.save_to_file_pub.publish(String(filename))

    def _loadFromFileButtonClicked(self):
        filename, _ = self.file_dialog.getOpenFileName(self._widget, caption="Load from file",
                                                       filter="Binary Files (*.bin);;Text Files (*.txt);;All Files (*)")
        self.load_from_file_pub.publish(String(filename))

    def _updateMeasurementCombo(self, msg: MeasurementIndices):
        combo = self._widget.measurement_idx_combo_box
        cur_data = None
        if combo.currentIndex() >= 0:
            d = combo.itemData(combo.currentIndex(), QtCore.Qt.UserRole)
            if isinstance(d, tuple) and len(d) == 2:
                cur_data = d

        blocker = QtCore.QSignalBlocker(combo)
        combo.clear()
        combo.addItem("latest", (-1, -1))

        for mi in msg.measurement_indices:
            text = f"{mi.measurement_idx} - {mi.measurement_id}"
            combo.addItem(text, (int(mi.measurement_idx), int(mi.measurement_id)))

        target = cur_data if cur_data is not None else (-1, -1)
        found = False
        for i in range(combo.count()):
            if combo.itemData(i, QtCore.Qt.UserRole) == target:
                combo.setCurrentIndex(i)
                found = True
                break
        if not found:
            combo.setCurrentIndex(0)

    def _onMeasurementComboChanged(self, _idx: int):
        combo = self._widget.measurement_idx_combo_box
        if combo.currentIndex() < 0:
            return
        data = combo.itemData(combo.currentIndex(), QtCore.Qt.UserRole)
        if not (isinstance(data, tuple) and len(data) == 2):
            return
        m_idx, m_id = data
        msg = MeasurementIndex()
        msg.measurement_idx = int(m_idx)
        msg.measurement_id = int(m_id)
        self.measurement_idx_pub.publish(msg)
        self._last_published_measurement = (msg.measurement_idx, msg.measurement_id)
        rospy.loginfo("measurement_idx_to_visualize: published (%d, %d)", msg.measurement_idx, msg.measurement_id)

    def _statusCallback(self, msg):
        self.last_status_update_time = datetime.now()
        self._widget.status_label.setText(msg.data)
