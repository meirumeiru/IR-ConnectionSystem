using System;
using System.Collections;
using System.Collections.Generic;

using UnityEngine;

using DockingFunctions;


namespace IR_ConnectionSystem.Module
{
	public class ModuleIRLEE : PartModule, IDockable, IModuleInfo, IResourceConsumer, IConstruction
	{
		// Settings

		[KSPField(isPersistant = false), SerializeField]
		public string nodeType = "LEE";

		[KSPField(isPersistant = false), SerializeField]
		private string nodeTypesAccepted = "GF";

		public HashSet<string> nodeTypesAcceptedS = null;


		[KSPField(isPersistant = false), SerializeField]
		public string nodeTransformName = "dockingNode";

		[KSPField(isPersistant = false), SerializeField]
		public string referenceAttachNode = ""; // if something is connected to this node, then the state is "Attached" (or "Pre-Attached" -> connected in the VAB/SPH)

		[KSPField(isPersistant = false), SerializeField]
		public Vector3 dockingOrientation = Vector3.up; // defines the direction of the docking port (when docked at a 0° angle, these local vectors of two ports point into the same direction)

		[KSPField(isPersistant = false), SerializeField]
		public int snapCount = 1;


		[KSPField(isPersistant = false), SerializeField]
		public float detectionDistance = 5f;

		[KSPField(isPersistant = false), SerializeField]
		public float approachingDistance = 0.3f;

		[KSPField(isPersistant = false), SerializeField]
		public float approachingAlignment = 15f;


		[KSPField(isPersistant = false), SerializeField]
		public float captureDistance = 0.06f;

		[KSPField(isPersistant = false), SerializeField]
		public float captureAlignment = 5f;

		[KSPField(isPersistant = false), SerializeField]
		public float captureAngle = 5f;

		[KSPField(isPersistant = true), SerializeField]
		public bool canAutoCapture = true;

		[KSPField(isPersistant = true), SerializeField]
		public bool autoCapture = false;


		[KSPField(isPersistant = false), SerializeField]
		public float capturingForce = 300f;

		[KSPField(isPersistant = false), SerializeField]
		public float captureBreakingForceFactor = 0.02f;

		[KSPField(isPersistant = false), SerializeField]
		public float captureBreakingDistance = 0.01f;

		[KSPField(isPersistant = false), SerializeField]
		public float captureBreakingAngle = 1.5f;

		[KSPField(isPersistant = false), SerializeField]
		public float capturingSpeedTranslation = 0.025f; // distance per second


		[KSPField(isPersistant = false), SerializeField]
		public float latchingForce = 10000f;

		[KSPField(isPersistant = false), SerializeField]
		public float latchingBreakingForceFactor = 0.2f;

		[KSPField(isPersistant = false), SerializeField]
		public float latchingBreakingDistance = 0.006f;

		[KSPField(isPersistant = false), SerializeField]
		public float latchingBreakingAngle = 1.0f;

		[KSPField(isPersistant = false), SerializeField]
		public float latchingSpeedRotation = 0.1f; // degrees per second

		[KSPField(isPersistant = false), SerializeField]
		public float latchingSpeedTranslation = 0.025f; // distance per second


		[KSPField(isPersistant = false), SerializeField]
		public float latchingDistance = 0.002f;

		[KSPField(isPersistant = false), SerializeField]
		public float latchingAlignment = 0.04f;

		[KSPField(isPersistant = false), SerializeField]
		public float latchingAngle = 0.04f;


		[KSPField(isPersistant = false), SerializeField]
		public bool canCrossfeed = true;

		[KSPField(isPersistant = true)]
		public bool crossfeed = true;


		[KSPField(isPersistant = false), SerializeField]
		private float electricChargeRequiredLatching = 1.5f;

		[KSPField(isPersistant = false), SerializeField]
		private float electricChargeRequiredReleasing = 0.5f;

		private PartResourceDefinition electricResource = null;

		// Docking and Status

		public Transform nodeTransform;

		public KerbalFSM fsm;

		public KFSMState st_active;			// "active" / "searching"

		public KFSMState st_approaching;	// port found

		public KFSMState st_capturing;		// we establish a first connection
		public KFSMState st_captured;		// we have a first, weak but stable and centered connection and the system is ready for orienting, pullback and latching

		public KFSMState st_latching;		// orienting and retracting in progress
		public KFSMState st_prelatched;		// ready to dock

		public KFSMState st_latchfailed;
		public KFSMState st_latched;		// docked

		public KFSMState st_released;		// after a capture or latch, the parts have been detached again -> maybe for an abort of the docking
		
		public KFSMState st_docked;			// docked or docked_to_same_vessel
		public KFSMState st_preattached;

		public KFSMState st_disabled;


		public KFSMEvent on_approach;
		public KFSMEvent on_distance;

		public KFSMEvent on_capturing;
		public KFSMEvent on_capture;

		public KFSMEvent on_latching;
		public KFSMEvent on_prelatch;

		public KFSMEvent on_latchfailed;

		public KFSMEvent on_latch;

		public KFSMEvent on_release;

		public KFSMEvent on_dock;
		public KFSMEvent on_undock;

		public KFSMEvent on_enable;
		public KFSMEvent on_disable;

		public KFSMEvent on_construction;

		// Sounds

			// option for later

		// Capturing / Docking

		public ModuleIRGF otherPort;
		public uint dockedPartUId;
		public uint dockedType; // defines which side this part is -> 0 = part, 1 = targetPart

		public DockedVesselInfo vesselInfo;

		private bool inCaptureDistance = false;

		private ConfigurableJoint joint;

		private Vector3 jointInitialPosition;

		private float jointBreakForce;
		private float jointBreakTorque;

		private Quaternion jointTargetRotation;
		private Vector3 jointTargetPosition;

		private float jointLastDistance;
		private float jointLastAlignment;

		private float progress;
		private float progressStep = 0.0005f;

		private int waitCounter;
		private int relaxCounter;

		// Packed / OnRails

		private int followOtherPort = 0;

		////////////////////////////////////////
		// Constructor

		public ModuleIRLEE()
		{
		}

		////////////////////////////////////////
		// Callbacks

		public override void OnAwake()
		{
#if DEBUG
	//		DebugInit();
#endif

			part.dockingPorts.AddUnique(this);


			electricResource = PartResourceLibrary.Instance.GetDefinition("ElectricCharge");

			if(consumedResources == null)
				consumedResources = new List<PartResourceDefinition>();
			else
				consumedResources.Clear();

			consumedResources.Add(electricResource);
		}

		public override void OnLoad(ConfigNode node)
		{
			base.OnLoad(node);

			if(node.HasValue("state"))
				DockStatus = node.GetValue("state");
			else
				DockStatus = "Inactive";

			if(node.HasValue("dockUId"))
				dockedPartUId = uint.Parse(node.GetValue("dockUId"));

			if(node.HasValue("dockedType"))
				dockedType = uint.Parse(node.GetValue("dockedType"));

			if(node.HasNode("DOCKEDVESSEL"))
			{
				vesselInfo = new DockedVesselInfo();
				vesselInfo.Load(node.GetNode("DOCKEDVESSEL"));
			}
		}

		public override void OnSave(ConfigNode node)
		{
			base.OnSave(node);

			node.AddValue("state", (string)(((fsm != null) && (fsm.Started)) ? fsm.currentStateName : DockStatus));

			node.AddValue("dockUId", dockedPartUId);

			node.AddValue("dockedType", dockedType);

			if(vesselInfo != null)
				vesselInfo.Save(node.AddNode("DOCKEDVESSEL"));
		}

		public override void OnStart(StartState state)
		{
			base.OnStart(state);

			nodeTypesAcceptedS = new HashSet<string>();

			string[] values = nodeTypesAccepted.Split(new char[2] { ',', ' ' }, StringSplitOptions.RemoveEmptyEntries);
			foreach(string s in values)
				nodeTypesAcceptedS.Add(s);

			if(state == StartState.Editor)
				return;

			GameEvents.onVesselGoOnRails.Add(OnPack);
			GameEvents.onVesselGoOffRails.Add(OnUnpack);

			GameEvents.OnEVAConstructionModePartDetached.Add(OnEVAConstructionModePartDetached);

			nodeTransform = part.FindModelTransform(nodeTransformName);
			if(!nodeTransform)
			{
				Logger.Log("No node transform found with name " + nodeTransformName, Logger.Level.Error);
				return;
			}

			StartCoroutine(WaitAndInitialize(state));

	//		StartCoroutine(WaitAndDisableDockingNode());
		}

		public IEnumerator WaitAndInitialize(StartState st)
		{
			yield return null;

			Events["TogglePort"].active = false;

			Events["AutoCapture"].active = false;
			Events["AutoCapture"].guiName = autoCapture ? "Capturing: Auto" : "Capturing: Manual";

			Events["Capture"].active = false;
			Events["Latch"].active = false;
			Events["Release"].active = false;
			Events["Restore"].active = false;

			Events["Dock"].active = false;
			Events["Undock"].active = false;

			if(!canCrossfeed) crossfeed = false;

			part.fuelCrossFeed = crossfeed;

			Events["EnableXFeed"].active = !crossfeed;
			Events["DisableXFeed"].active = crossfeed;

			if(dockedPartUId != 0)
			{
				Part otherPart;

				while(!(otherPart = FlightGlobals.FindPartByID(dockedPartUId)))
					yield return null;

				otherPort = otherPart.GetComponent<ModuleIRGF>();

				otherPort.otherPort = this;
				otherPort.dockedPartUId = part.flightID;
			}

			if((DockStatus == "Inactive")
			|| ((DockStatus == "Attached") && (otherPort == null)))
			{
				if(referenceAttachNode != string.Empty)
				{
					AttachNode node = part.FindAttachNode(referenceAttachNode);
					if((node != null) && node.attachedPart)
					{
						ModuleIRGF _otherPort = node.attachedPart.GetComponent<ModuleIRGF>();

						if(_otherPort)
						{
							otherPort = _otherPort;
							dockedPartUId = otherPort.part.flightID;

							DockStatus = "Attached";
							otherPort.DockStatus = "Attached";
						}
					}
				}
			}

			SetupFSM();

			if((DockStatus == "Approaching")
			|| (DockStatus == "Capturing")
			|| (DockStatus == "Captured")
			|| (DockStatus == "Latching")
			|| (DockStatus == "Pre Latched")
			|| (DockStatus == "Latched")
			|| (DockStatus == "Released"))
			{
				if(otherPort != null)
				{
					while(!otherPort.part.started || (otherPort.fsm == null) || (!otherPort.fsm.Started))
						yield return null;
				}
			}

			if(DockStatus == "Pre Latched")
				DockStatus = "Latching";

			if((DockStatus == "Captured")
			|| (DockStatus == "Latching"))
			{
				BuildJoint();
				CalculateJointTarget();

				ConfigureJointWeak();
			}

		//	if(DockStatus == "Latched") -> do nothing special

			if(DockStatus == "Docked")
			{
				otherPort.DockStatus = "Docked";

				if(dockedType == 0)
					DockingHelper.OnLoad(this, vesselInfo, otherPort, otherPort.vesselInfo);
				else
					DockingHelper.OnLoad(otherPort, otherPort.vesselInfo, this, vesselInfo);
			}

			fsm.StartFSM(DockStatus);

			if(joint)
			{
				if(Vessel.GetDominantVessel(vessel, otherPort.vessel) == otherPort.vessel)
					followOtherPort = VesselPositionManager.Register(part, otherPort.part);
				else
					followOtherPort = VesselPositionManager.Register(otherPort.part, part);
			}
		}
	/*
		public IEnumerator WaitAndDisableDockingNode()
		{
			ModuleDockingNode DockingNode = part.FindModuleImplementing<ModuleDockingNode>();

			if(DockingNode)
			{
				while((DockingNode.fsm == null) || (!DockingNode.fsm.Started))
					yield return null;

				DockingNode.fsm.RunEvent(DockingNode.on_disable);
			}
		}
	*/
		public void OnDestroy()
		{
			GameEvents.onVesselGoOnRails.Remove(OnPack);
			GameEvents.onVesselGoOffRails.Remove(OnUnpack);

			GameEvents.OnEVAConstructionModePartDetached.Remove(OnEVAConstructionModePartDetached);
		}

		private void OnPack(Vessel v)
		{
			if(vessel == v)
			{
				if(joint)
				{
					if(Vessel.GetDominantVessel(vessel, otherPort.vessel) == otherPort.vessel)
						followOtherPort = VesselPositionManager.Register(part, otherPort.part);
					else
						followOtherPort = VesselPositionManager.Register(otherPort.part, part);
				}
			}
		}

		private void OnUnpack(Vessel v)
		{
			if(vessel == v)
			{
				if(followOtherPort != 0)
				{
					VesselPositionManager.Unregister(followOtherPort);
					followOtherPort = 0;
				}
			}
		}

		private void OnEVAConstructionModePartDetached(Vessel v, Part p)
		{
			if(part == p)
			{
				if(otherPort)
				{
					otherPort.otherPort = null;
					otherPort.dockedPartUId = 0;
					otherPort.fsm.RunEvent(otherPort.on_construction);
				}

				otherPort = null;
				dockedPartUId = 0;
				fsm.RunEvent(on_construction);
			}
		}

		////////////////////////////////////////
		// Functions

		public void SetupFSM()
		{
			fsm = new KerbalFSM();

			st_active = new KFSMState("Ready");
			st_active.OnEnter = delegate(KFSMState from)
			{
				otherPort = null;
				dockedPartUId = 0;

				Events["TogglePort"].guiName = "Deactivate End Effector";
				Events["TogglePort"].active = true;

				Events["AutoCapture"].guiName = autoCapture ? "Capturing: Auto" : "Capturing: Manual";
				Events["AutoCapture"].active = canAutoCapture;

				DockStatus = st_active.name;
			};
			st_active.OnFixedUpdate = delegate
			{
				float distance; float alignment;

				for(int i = 0; i < FlightGlobals.VesselsLoaded.Count; i++)
				{
					Vessel vessel = FlightGlobals.VesselsLoaded[i];

					if(vessel.packed
						/*|| (vessel == part.vessel)*/) // no docking to ourself is possible
						continue;

					for(int j = 0; j < vessel.dockingPorts.Count; j++)
					{
						PartModule partModule = vessel.dockingPorts[j];

						if((partModule.part == null)
						/*|| (partModule.part == part)*/ // no docking to ourself is possible
						|| (partModule.part.State == PartStates.DEAD))
							continue;

						ModuleIRGF _otherPort = partModule.GetComponent<ModuleIRGF>();

						if(_otherPort == null)
							continue;

						if(!nodeTypesAcceptedS.Contains(_otherPort.nodeType)
						|| !_otherPort.nodeTypesAcceptedS.Contains(nodeType))
							continue;

						if(_otherPort.fsm.CurrentState != _otherPort.st_passive)
							continue;

						distance = (_otherPort.nodeTransform.position - nodeTransform.position).magnitude;

						if(distance < detectionDistance)
						{
							DockDistance = distance.ToString();

							alignment = Vector3.Angle(nodeTransform.forward, -_otherPort.nodeTransform.forward);

							if((alignment <= approachingAlignment) && (distance <= approachingDistance))
							{
								DockAlignment = alignment.ToString();
								DockAngle = "-";

								// we don't expect to see multiple matching ports in the same area
								// that's why we don't continue to search and simply take the first we find

								otherPort = _otherPort;
								dockedPartUId = otherPort.part.flightID;

								fsm.RunEvent(on_approach);
								otherPort.fsm.RunEvent(otherPort.on_approach_passive);
								return;
							}
						}
					}
				}

				DockDistance = "-";
				DockAlignment = "-";
				DockAngle = "-";
			};
			st_active.OnLeave = delegate(KFSMState to)
			{
				if(to != st_disabled)
					Events["TogglePort"].active = false;
			};
			fsm.AddState(st_active);

			st_approaching = new KFSMState("Approaching");
			st_approaching.OnEnter = delegate(KFSMState from)
			{
				Events["AutoCapture"].guiName = autoCapture ? "Capturing: Auto" : "Capturing: Manual";
				Events["AutoCapture"].active = canAutoCapture;

				inCaptureDistance = false;

				otherPort.otherPort = this;
				otherPort.dockedPartUId = part.flightID;

			//	otherPort.fsm.RunEvent(otherPort.on_approach_passive); -> is done manually

				DockStatus = st_approaching.name;
			};
			st_approaching.OnFixedUpdate = delegate
			{
				float distance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;
				float alignment = Vector3.Angle(nodeTransform.forward, -otherPort.nodeTransform.forward);
				float angle = CalculateAngle();

				DockDistance = distance.ToString();
				DockAlignment = alignment.ToString();
				DockAngle = angle.ToString();

				if((distance < captureDistance)
				&& (alignment < captureAlignment)
				&& (Mathf.Abs(angle) <= captureAngle))
				{
					if(autoCapture)
					{
						fsm.RunEvent(on_capturing);
						otherPort.fsm.RunEvent(otherPort.on_capture_passive);

						return;
					}

					if(!inCaptureDistance)
						Events["Capture"].active = true;

					inCaptureDistance = true;

					return;
				}

				if(inCaptureDistance)
					Events["Capture"].active = false;

				inCaptureDistance = false;
				
				if(distance < 1.5f * approachingDistance)
				{
					if(alignment <= approachingAlignment)
						return;
				}

				otherPort.fsm.RunEvent(otherPort.on_distance_passive);
				fsm.RunEvent(on_distance);
			};
			st_approaching.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_approaching);

			st_capturing = new KFSMState("Capturing");
			st_capturing.OnEnter = delegate(KFSMState from)
			{
				Events["AutoCapture"].active = false;

				Events["Capture"].active = false;
				Events["Release"].active = true;

				BuildJoint();
				CalculateJointTarget();

				ConfigureJointWeak();

				jointLastDistance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;

			//	otherPort.fsm.RunEvent(otherPort.on_capture_passive); -> already done

				DockStatus = st_capturing.name;
			};
			st_capturing.OnFixedUpdate = delegate
			{
				if(electricChargeRequiredLatching > 0f)
				{
					double amountRequested = electricChargeRequiredLatching * 0.2f * TimeWarp.fixedDeltaTime;

					if(part.RequestResource(electricResource.id, amountRequested) < 0.95f * amountRequested)
					{
						fsm.RunEvent(on_latchfailed);
						return;
					}
				}

				// distance from axis
				Vector3 diff = otherPort.nodeTransform.position - nodeTransform.position;
				Vector3 diffp = Vector3.ProjectOnPlane(diff, nodeTransform.forward);
				Vector3 diffpl = Quaternion.Inverse(joint.transform.rotation) * diffp;

				float distance = diff.magnitude;

				if(Mathf.Abs(jointLastDistance - distance) > captureBreakingDistance)
				{
					fsm.RunEvent(on_latchfailed);
					return;
				}

				jointLastDistance = distance;

				if(diffpl.magnitude >= capturingSpeedTranslation * TimeWarp.fixedDeltaTime)
					joint.targetPosition -= diffpl.normalized * capturingSpeedTranslation * TimeWarp.fixedDeltaTime;
				else
				{
					joint.targetPosition -= diffpl;

					fsm.RunEvent(on_capture);
				//	otherPort.fsm.RunEvent(otherPort.on_capture_passive); -> already done
				}
			};
			st_capturing.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_capturing);

			st_captured = new KFSMState("Captured");
			st_captured.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;
				Events["Latch"].active = true;

				jointLastDistance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;
				jointLastAlignment = Vector3.Angle(nodeTransform.forward, -otherPort.nodeTransform.forward);

				DockStatus = st_captured.name;
			};
			st_captured.OnFixedUpdate = delegate
			{
				float distance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;
				float alignment = Vector3.Angle(nodeTransform.forward, -otherPort.nodeTransform.forward);

				if((Mathf.Abs(jointLastDistance - distance) > captureBreakingDistance)
				|| (Mathf.Abs(jointLastAlignment - alignment) > captureBreakingAngle))
					fsm.RunEvent(on_latchfailed);
			};
			st_captured.OnLeave = delegate(KFSMState to)
			{
				if(to != st_latching)
					Events["Release"].active = false;

				Events["Latch"].active = false;
			};
			fsm.AddState(st_captured);
		
			st_latching = new KFSMState("Latching");
			st_latching.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;

				jointLastDistance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;
				jointLastAlignment = Vector3.Angle(nodeTransform.forward, -otherPort.nodeTransform.forward);

				float latchingDuration = Math.Max(
						(nodeTransform.position - otherPort.nodeTransform.position).magnitude / latchingSpeedTranslation,
						(Quaternion.Angle(jointTargetRotation, Quaternion.identity) / latchingSpeedRotation));

				if(float.IsNaN(latchingDuration) || float.IsInfinity(latchingDuration))
					latchingDuration = 10;

				progress = 1;
				progressStep = TimeWarp.fixedDeltaTime / latchingDuration;

				jointInitialPosition = joint.targetPosition;

				DockStatus = st_latching.name;
			};
			st_latching.OnFixedUpdate = delegate
			{
				if(electricChargeRequiredLatching > 0f)
				{
					double amountRequested = electricChargeRequiredLatching * TimeWarp.fixedDeltaTime;

					if(part.RequestResource(electricResource.id, amountRequested) < 0.95f * amountRequested)
					{
						fsm.RunEvent(on_latchfailed);
						return;
					}
				}

				float distance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;
				float alignment = Vector3.Angle(nodeTransform.forward, -otherPort.nodeTransform.forward);
				float angle = CalculateAngle();

				DockDistance = distance.ToString();
				DockAlignment = alignment.ToString();
				DockAngle = angle.ToString();

				if((jointLastDistance - distance > latchingBreakingDistance)
				|| (jointLastAlignment - alignment > latchingBreakingAngle))
				{
					fsm.RunEvent(on_latchfailed);
					return;
				}

				if((joint.currentForce.sqrMagnitude > jointBreakForce * jointBreakForce)
				|| (joint.currentTorque.sqrMagnitude > jointBreakTorque * jointBreakTorque))
				{
					fsm.RunEvent(on_latchfailed);
					return;
				}

				jointLastDistance = distance;
				jointLastAlignment = alignment;

				if((distance > captureDistance * 1.04f)
				|| (angle > captureAlignment * 1.04f)
				|| (Mathf.Abs(angle) > captureAngle * 1.04f))
				{
					fsm.RunEvent(on_latchfailed);
					return;
				}

				if(progress > progressStep)
				{
					progress -= progressStep;

					float factor = (1f - progress) * latchingBreakingForceFactor + progress * captureBreakingForceFactor;

					jointBreakForce = Mathf.Min(part.breakingForce, otherPort.part.breakingForce) *
						factor;

					jointBreakTorque = Mathf.Min(part.breakingTorque, otherPort.part.breakingTorque) *
						factor;

					float force = (1f - progress) * latchingForce + progress * capturingForce;

					JointDrive angularDrive = new JointDrive { maximumForce = force, positionSpring = 60000f, positionDamper = 0f };
					joint.angularXDrive = joint.angularYZDrive = joint.slerpDrive = angularDrive;

					JointDrive linearDrive = new JointDrive { maximumForce = force, positionSpring = PhysicsGlobals.JointForce, positionDamper = 0f };
					joint.xDrive = joint.yDrive = joint.zDrive = linearDrive;

					joint.targetRotation = Quaternion.Slerp(jointTargetRotation, Quaternion.identity, progress);
					joint.targetPosition = Vector3.Lerp(jointTargetPosition, jointInitialPosition, progress);
				}
				else
					fsm.RunEvent(on_prelatch);
			};
			st_latching.OnLeave = delegate(KFSMState to)
			{
				if(to != st_prelatched)
					Events["Release"].active = false;
			};
			fsm.AddState(st_latching);

			st_prelatched = new KFSMState("Pre Latched");
			st_prelatched.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;

				jointBreakForce = Mathf.Min(part.breakingForce, otherPort.part.breakingForce) *
					latchingBreakingForceFactor;

				jointBreakTorque = Mathf.Min(part.breakingTorque, otherPort.part.breakingTorque) *
						latchingBreakingForceFactor;

				JointDrive angularDrive = new JointDrive { maximumForce = latchingForce, positionSpring = 60000f, positionDamper = 0f };
				joint.angularXDrive = joint.angularYZDrive = joint.slerpDrive = angularDrive;

				JointDrive linearDrive = new JointDrive { maximumForce = latchingForce, positionSpring = PhysicsGlobals.JointForce, positionDamper = 0f };
				joint.xDrive = joint.yDrive = joint.zDrive = linearDrive;

				joint.targetRotation = jointTargetRotation;
				joint.targetPosition = jointTargetPosition;

				waitCounter = 1000;
				progress = 1;
				relaxCounter = 10;

				DockStatus = st_prelatched.name;
			};
			st_prelatched.OnFixedUpdate = delegate
			{
				if(electricChargeRequiredLatching > 0f)
				{
					double amountRequested = electricChargeRequiredLatching * TimeWarp.fixedDeltaTime;

					if(part.RequestResource(electricResource.id, amountRequested) < 0.95f * amountRequested)
					{
						fsm.RunEvent(on_latchfailed);
						return;
					}
				}

				float distance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;
				float alignment = Vector3.Angle(nodeTransform.forward, -otherPort.nodeTransform.forward);
				float angle = CalculateAngle();

				DockDistance = distance.ToString();
				DockAlignment = alignment.ToString();
				DockAngle = angle.ToString();

				if((jointLastDistance - distance > latchingBreakingDistance)
				|| (jointLastAlignment - alignment > latchingBreakingAngle))
				{
					fsm.RunEvent(on_latchfailed);
					return;
				}

				if((joint.currentForce.sqrMagnitude > jointBreakForce * jointBreakForce)
				|| (joint.currentTorque.sqrMagnitude > jointBreakTorque * jointBreakTorque))
				{
					fsm.RunEvent(on_latchfailed);
					return;
				}

				jointLastDistance = distance;
				jointLastAlignment = alignment;

				if(waitCounter > 0)
				{
					if((distance < latchingDistance)
					&& (alignment < latchingAlignment)
					&& (angle < latchingAngle))
						waitCounter = 0;
					else
					{
						if(--waitCounter <= 0)
							fsm.RunEvent(on_latchfailed);
						return;
					}
				}

				if((distance > latchingDistance * 1.04f)
				|| (angle > latchingAlignment * 1.04f)
				|| (Mathf.Abs(angle) > latchingAngle * 1.04f))
				{
					fsm.RunEvent(on_latchfailed);
					return;
				}

				if(progress > 0f)
				{
					// factor 0.6f used in this function -> a bit more than half of the force is available before fully latched

					progress = (progress > 0.05f) ? progress - 0.05f : 0f;

					float factor = (1f - progress) * 4f * 0.6f + progress * latchingBreakingForceFactor;

					jointBreakForce = Mathf.Min(part.breakingForce, otherPort.part.breakingForce) *
						factor;

					jointBreakTorque = Mathf.Min(part.breakingTorque, otherPort.part.breakingTorque) *
						factor;

					float force = (1f - progress) * PhysicsGlobals.JointForce * 0.6f + progress * latchingForce;

					JointDrive angularDrive = new JointDrive { maximumForce = force, positionSpring = 60000f, positionDamper = 0f };
					joint.angularXDrive = joint.angularYZDrive = joint.slerpDrive = angularDrive;

					JointDrive linearDrive = new JointDrive { maximumForce = force, positionSpring = PhysicsGlobals.JointForce, positionDamper = 0f };
					joint.xDrive = joint.yDrive = joint.zDrive = linearDrive;

					return;
				}

				if(--relaxCounter < 0)
				{
					fsm.RunEvent(on_latch);
					otherPort.fsm.RunEvent(otherPort.on_latch_passive);
				}
			};
			st_prelatched.OnLeave = delegate(KFSMState to)
			{
				if(to != st_latched)
					Events["Release"].active = false;
			};
			fsm.AddState(st_prelatched);
		
			st_latchfailed = new KFSMState("Latch Failed");
			st_latchfailed.OnEnter = delegate(KFSMState from)
			{
				if(otherPort != null)
					otherPort.fsm.RunEvent(otherPort.on_release_passive);

				DestroyJoint();

				waitCounter = 200;

				DockStatus = st_latchfailed.name;
			};
			st_latchfailed.OnFixedUpdate = delegate
			{
				if(--waitCounter < 0)
					fsm.RunEvent(on_approach);
			};
			st_latchfailed.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_latchfailed);

			st_latched = new KFSMState("Latched");
			st_latched.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;

				Events["Dock"].active = true;
				Events["Undock"].active = false;

				if(joint == null)
				{
					BuildJoint();
					CalculateJointTarget();
				}

				joint.targetRotation = jointTargetRotation;
				joint.targetPosition = jointTargetPosition;

				ConfigureJointRigid();

				DockStatus = st_latched.name;
			};
			st_latched.OnFixedUpdate = delegate
			{
			};
			st_latched.OnLeave = delegate(KFSMState to)
			{
				Events["Release"].active = false;

				Events["Dock"].active = false;
			};
			fsm.AddState(st_latched);

			st_released = new KFSMState("Released");
			st_released.OnEnter = delegate(KFSMState from)
			{
				DestroyJoint();

				Events["Restore"].active = true;

			//	if(otherPort != null)
			//		otherPort.fsm.RunEvent(otherPort.on_release_passive); -> already done

				DockStatus = st_released.name;
			};
			st_released.OnFixedUpdate = delegate
			{
				float distance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;

				DockDistance = distance.ToString();
				DockAlignment = "-";
				DockAngle = "-";

				if(distance > 1.1f * approachingDistance)
					fsm.RunEvent(on_distance);
			};
			st_released.OnLeave = delegate(KFSMState to)
			{
				Events["Restore"].active = false;
			};
			fsm.AddState(st_released);
		
			st_docked = new KFSMState("Docked");
			st_docked.OnEnter = delegate(KFSMState from)
			{
				Events["Undock"].active = true;

				DockStatus = st_docked.name;
			};
			st_docked.OnFixedUpdate = delegate
			{
			};
			st_docked.OnLeave = delegate(KFSMState to)
			{
				Events["Undock"].active = false;
			};
			fsm.AddState(st_docked);

			st_preattached = new KFSMState("Attached");
			st_preattached.OnEnter = delegate(KFSMState from)
			{
				Events["Undock"].active = true;

				DockStatus = st_preattached.name;
			};
			st_preattached.OnFixedUpdate = delegate
			{
			};
			st_preattached.OnLeave = delegate(KFSMState to)
			{
				Events["Undock"].active = false;
			};
			fsm.AddState(st_preattached);

			st_disabled = new KFSMState("Inactive");
			st_disabled.OnEnter = delegate(KFSMState from)
			{
				Events["TogglePort"].guiName = "Activate End Effector";
				Events["TogglePort"].active = true;

				Events["AutoCapture"].active = false;

				DockStatus = st_disabled.name;
			};
			st_disabled.OnFixedUpdate = delegate
			{
			};
			st_disabled.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_disabled);


			on_approach = new KFSMEvent("Approaching");
			on_approach.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_approach.GoToStateOnEvent = st_approaching;
			fsm.AddEvent(on_approach, st_active, st_latchfailed);

			on_distance = new KFSMEvent("Distancing");
			on_distance.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_distance.GoToStateOnEvent = st_active;
			fsm.AddEvent(on_distance, st_approaching, st_released);

			on_capturing = new KFSMEvent("Capture");
			on_capturing.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_capturing.GoToStateOnEvent = st_capturing;
			fsm.AddEvent(on_capturing, st_approaching);

			on_capture = new KFSMEvent("Captured");
			on_capture.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_capture.GoToStateOnEvent = st_captured;
			fsm.AddEvent(on_capture, st_capturing);
			
			on_latching = new KFSMEvent("Latch");
			on_latching.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latching.GoToStateOnEvent = st_latching;
			fsm.AddEvent(on_latching, st_captured);

			on_prelatch = new KFSMEvent("Pre Latch");
			on_prelatch.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_prelatch.GoToStateOnEvent = st_prelatched;
			fsm.AddEvent(on_prelatch, st_latching);

			on_latchfailed = new KFSMEvent("Latching Failed");
			on_latchfailed.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latchfailed.GoToStateOnEvent = st_latchfailed;
			fsm.AddEvent(on_latchfailed, st_capturing, st_captured, st_latching, st_prelatched);

			on_latch = new KFSMEvent("Latched");
			on_latch.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latch.GoToStateOnEvent = st_latched;
			fsm.AddEvent(on_latch, st_prelatched);


			on_release = new KFSMEvent("Released");
			on_release.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_release.GoToStateOnEvent = st_released;
			fsm.AddEvent(on_release, st_captured, st_latched);


			on_dock = new KFSMEvent("Perform docking");
			on_dock.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_dock.GoToStateOnEvent = st_docked;
			fsm.AddEvent(on_dock, st_latched);

			on_undock = new KFSMEvent("Undock");
			on_undock.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_undock.GoToStateOnEvent = st_latched;
			fsm.AddEvent(on_undock, st_docked, st_preattached);


			on_enable = new KFSMEvent("Enable");
			on_enable.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_enable.GoToStateOnEvent = st_active;
			fsm.AddEvent(on_enable, st_disabled);

			on_disable = new KFSMEvent("Disable");
			on_disable.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_disable.GoToStateOnEvent = st_disabled;
			fsm.AddEvent(on_disable, st_active);


			on_construction = new KFSMEvent("Construction");
			on_construction.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_construction.GoToStateOnEvent = st_disabled;
			fsm.AddEvent(on_construction, st_active, st_approaching, st_capturing, st_captured, st_latching, st_prelatched, st_latched, st_released, st_docked, st_preattached);
		}

		private float CalculateAngle()
		{
			Vector3 tvref = nodeTransform.TransformDirection(dockingOrientation);
			Vector3 tv = otherPort.nodeTransform.TransformDirection(otherPort.dockingOrientation);
			float angle = Vector3.SignedAngle(tvref, tv, -nodeTransform.forward);

			angle = 360f + angle - (180f / snapCount);
			angle %= (360f / snapCount);
			angle -= (180f / snapCount);

			return angle;
		}

		private void BuildJoint()
		{
			joint = gameObject.AddComponent<ConfigurableJoint>();

			joint.connectedBody = otherPort.part.Rigidbody;

			DockDistance = "-";
			DockAlignment = "-";
			DockAngle = "-";
		}

		// modifies the joint so that it is moveable
		private void ConfigureJointWeak()
		{
			jointBreakForce = Mathf.Min(part.breakingForce, otherPort.part.breakingForce) *
				captureBreakingForceFactor;

			jointBreakTorque = Mathf.Min(part.breakingTorque, otherPort.part.breakingTorque) *
				captureBreakingForceFactor;

			joint.xMotion = joint.yMotion = joint.zMotion = ConfigurableJointMotion.Free;
			joint.angularXMotion = joint.angularYMotion = joint.angularZMotion = ConfigurableJointMotion.Free;

			SoftJointLimit angularLimit = default(SoftJointLimit);
			angularLimit.bounciness = 0f;

			SoftJointLimitSpring angularLimitSpring = default(SoftJointLimitSpring);
			angularLimitSpring.spring = 0f;
			angularLimitSpring.damper = 0f;

			joint.highAngularXLimit = angularLimit;
			joint.lowAngularXLimit = angularLimit;
			joint.angularYLimit = angularLimit;
			joint.angularZLimit = angularLimit;
			joint.angularXLimitSpring = angularLimitSpring;
			joint.angularYZLimitSpring = angularLimitSpring;

			SoftJointLimit linearJointLimit = default(SoftJointLimit);
			linearJointLimit.limit = 1f;
			linearJointLimit.bounciness = 0f;

			SoftJointLimitSpring linearJointLimitSpring = default(SoftJointLimitSpring);
			linearJointLimitSpring.damper = 0f;
			linearJointLimitSpring.spring = 0f;

			joint.linearLimit = linearJointLimit;
			joint.linearLimitSpring = linearJointLimitSpring;

			JointDrive angularDrive = new JointDrive { maximumForce = capturingForce, positionSpring = 60000f, positionDamper = 0f };
			joint.angularXDrive = joint.angularYZDrive = angularDrive; 

			JointDrive linearDrive = new JointDrive { maximumForce = capturingForce, positionSpring = PhysicsGlobals.JointForce, positionDamper = 0f };
			joint.xDrive = joint.yDrive = joint.zDrive = linearDrive;

			joint.breakForce = float.MaxValue;
			joint.breakTorque = float.MaxValue;
		}

		// modifies the joint so that it has the same settings like a real docking joint
		private void ConfigureJointRigid()
		{
		//	float stackNodeFactor = 2f;
		//	float srfNodeFactor = 0.8f;

		//	float breakingForceModifier = 1f;
		//	float breakingTorqueModifier = 1f;

		//	float attachNodeSize = 1f;

			jointBreakForce = Mathf.Min(part.breakingForce, otherPort.part.breakingForce) *
				4f;
		//		breakingForceModifier *
		//		(attachNodeSize + 1f) * (part.attachMode == AttachModes.SRF_ATTACH ? srfNodeFactor : stackNodeFactor);

			jointBreakTorque = Mathf.Min(part.breakingTorque, otherPort.part.breakingTorque) *
				4f;
		//		breakingTorqueModifier *
		//		(attachNodeSize + 1f) * (part.attachMode == AttachModes.SRF_ATTACH ? srfNodeFactor : stackNodeFactor);

			joint.xMotion = joint.yMotion = joint.zMotion = ConfigurableJointMotion.Limited;
			joint.angularXMotion = joint.angularYMotion = joint.angularZMotion = ConfigurableJointMotion.Limited;

			SoftJointLimit angularLimit = default(SoftJointLimit);
			angularLimit.bounciness = 0f;

			SoftJointLimitSpring angularLimitSpring = default(SoftJointLimitSpring);
			angularLimitSpring.spring = 0f;
			angularLimitSpring.damper = 0f;

			joint.highAngularXLimit = angularLimit;
			joint.lowAngularXLimit = angularLimit;
			joint.angularYLimit = angularLimit;
			joint.angularZLimit = angularLimit;
			joint.angularXLimitSpring = angularLimitSpring;
			joint.angularYZLimitSpring = angularLimitSpring;

			SoftJointLimit linearJointLimit = default(SoftJointLimit);
			linearJointLimit.limit = 1f;
			linearJointLimit.bounciness = 0f;

			SoftJointLimitSpring linearJointLimitSpring = default(SoftJointLimitSpring);
			linearJointLimitSpring.damper = 0f;
			linearJointLimitSpring.spring = 0f;

			joint.linearLimit = linearJointLimit;
			joint.linearLimitSpring = linearJointLimitSpring;

			JointDrive angularDrive = new JointDrive { maximumForce = PhysicsGlobals.JointForce, positionSpring = 60000f, positionDamper = 0f };
			joint.angularXDrive = joint.angularYZDrive = angularDrive; 

			JointDrive linearDrive = new JointDrive { maximumForce = PhysicsGlobals.JointForce, positionSpring = PhysicsGlobals.JointForce, positionDamper = 0f };
			joint.xDrive = joint.yDrive = joint.zDrive = linearDrive;

			joint.breakForce = jointBreakForce;
			joint.breakTorque = jointBreakTorque;
		}

		private void CalculateJointTarget()
		{
			Vector3 targetPosition; Quaternion targetRotation;
			DockingHelper.CalculateLatchingPositionAndRotation(this, otherPort, out targetPosition, out targetRotation);

			// invert both values
			jointTargetPosition = -transform.InverseTransformPoint(targetPosition);
			jointTargetRotation = Quaternion.Inverse(Quaternion.Inverse(transform.rotation) * targetRotation);
		}

		private void DestroyJoint()
		{
			// Joint
			if(joint)
				Destroy(joint);
			joint = null;

			// for some rare cases
			vessel.ResetRBAnchor();
			if(otherPort) otherPort.vessel.ResetRBAnchor();
		}

		////////////////////////////////////////
		// Update-Functions

		public void FixedUpdate()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(vessel && !vessel.packed)
				{
					if((fsm != null) && fsm.Started)
						fsm.FixedUpdateFSM();
				}
			}
		}

		public void Update()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(vessel && !vessel.packed)
				{
					if((fsm != null) && fsm.Started)
						fsm.UpdateFSM();
				}
			}
		}

		public void LateUpdate()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(vessel && !vessel.packed)
				{
					if((fsm != null) && fsm.Started)
						fsm.LateUpdateFSM();
				}
			}
		}

		////////////////////////////////////////
		// Context Menu

		[KSPField(guiName = "LEE status", isPersistant = false, guiActive = true, guiActiveUnfocused = true, unfocusedRange = 20)]
		public string DockStatus = "Inactive";

		[KSPField(guiName = "LEE distance", isPersistant = false, guiActive = true)]
		public string DockDistance;

		[KSPField(guiName = "LEE alignment", isPersistant = false, guiActive = true)]
		public string DockAlignment;

		[KSPField(guiName = "LEE angle", isPersistant = false, guiActive = true)]
		public string DockAngle;

		public void Enable()
		{
			fsm.RunEvent(on_enable);
		}

		public void Disable()
		{
			fsm.RunEvent(on_disable);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, guiName = "Deactivate End Effector")]
		public void TogglePort()
		{
			if(fsm.CurrentState == st_disabled)
				fsm.RunEvent(on_enable);
			else
				fsm.RunEvent(on_disable);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, guiName = "Capturing: Manual")]
		public void AutoCapture()
		{
			autoCapture = !autoCapture;
			Events["AutoCapture"].guiName = autoCapture ? "Capturing: Auto" : "Capturing: Manual";
		}

		// first connection and centering (no rotation)
		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Capture")]
		public void Capture()
		{
			fsm.RunEvent(on_capturing);
			otherPort.fsm.RunEvent(otherPort.on_capture_passive);
		}

		// pull back and rotation (at the same time)
		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Latch")]
		public void Latch()
		{
			fsm.RunEvent(on_latching);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Release")]
		public void Release()
		{
			if(fsm.CurrentState == st_latched)
			{
				if(electricChargeRequiredReleasing > 0f)
				{
					if(part.RequestResource(electricResource.id, electricChargeRequiredReleasing, false) < 0.95f * electricChargeRequiredReleasing)
						return;
				}
			}

			if(otherPort != null)
				otherPort.fsm.RunEvent(otherPort.on_release_passive);

			fsm.RunEvent(on_release);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Restore")]
		public void Restore()
		{
			fsm.RunEvent(on_distance);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Dock")]
		public void Dock()
		{
			Debug.Log("Docking to vessel " + otherPort.vessel.GetDisplayName(), gameObject);

			dockedPartUId = otherPort.part.flightID;

			otherPort.otherPort = this;
			otherPort.dockedPartUId = part.flightID;

			DockingHelper.SaveCameraPosition(part);
			DockingHelper.SuspendCameraSwitch(10);

			if(otherPort.vessel == Vessel.GetDominantVessel(vessel, otherPort.vessel))
				DockingHelper.DockVessels(this, otherPort);
			else
				DockingHelper.DockVessels(otherPort, this);

			DockingHelper.RestoreCameraPosition(part);

			Destroy(joint);
			joint = null;

			fsm.RunEvent(on_dock);
			otherPort.fsm.RunEvent(otherPort.on_dock_passive);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, externalToEVAOnly = true, unfocusedRange = 2f, guiName = "#autoLOC_6001445")]
		public void Undock()
		{
			Vessel oldvessel = vessel;
			uint referenceTransformId = vessel.referenceTransformId;

			DockingHelper.SaveCameraPosition(part);
			DockingHelper.SuspendCameraSwitch(10);

			DockingHelper.UndockVessels(this, otherPort);

			DockingHelper.RestoreCameraPosition(part);

			otherPort.fsm.RunEvent(otherPort.on_undock_passive);
			fsm.RunEvent(on_undock);

			if(oldvessel == FlightGlobals.ActiveVessel)
			{
				if(vessel[referenceTransformId] == null)
					StartCoroutine(WaitAndSwitchFocus());
			}
		}

		public IEnumerator WaitAndSwitchFocus()
		{
			yield return null;

			DockingHelper.SaveCameraPosition(part);

			FlightGlobals.ForceSetActiveVessel(vessel);
			FlightInputHandler.SetNeutralControls();

			DockingHelper.RestoreCameraPosition(part);
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "#autoLOC_236028")]
		public void EnableXFeed()
		{
			Events["EnableXFeed"].active = false;
			Events["DisableXFeed"].active = true;
			bool fuelCrossFeed = part.fuelCrossFeed;
			part.fuelCrossFeed = (crossfeed = true);
			if(fuelCrossFeed != crossfeed)
				GameEvents.onPartCrossfeedStateChange.Fire(part);
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "#autoLOC_236030")]
		public void DisableXFeed()
		{
			Events["EnableXFeed"].active = true;
			Events["DisableXFeed"].active = false;
			bool fuelCrossFeed = part.fuelCrossFeed;
			part.fuelCrossFeed = (crossfeed = false);
			if(fuelCrossFeed != crossfeed)
				GameEvents.onPartCrossfeedStateChange.Fire(part);
		}

		////////////////////////////////////////
		// Actions

		[KSPAction("Enable")]
		public void EnableAction(KSPActionParam param)
		{ Enable(); }

		[KSPAction("Disable")]
		public void DisableAction(KSPActionParam param)
		{ Disable(); }

		[KSPAction("Dock", activeEditor = false)]
		public void DockAction(KSPActionParam param)
		{ Dock(); }

		[KSPAction("#autoLOC_6001444", activeEditor = false)]
		public void UndockAction(KSPActionParam param)
		{ Undock(); }

		[KSPAction("#autoLOC_236028")]
		public void EnableXFeedAction(KSPActionParam param)
		{ EnableXFeed(); }

		[KSPAction("#autoLOC_236030")]
		public void DisableXFeedAction(KSPActionParam param)
		{ DisableXFeed(); }

		[KSPAction("#autoLOC_236032")]
		public void ToggleXFeedAction(KSPActionParam param)
		{
			if(crossfeed)
				DisableXFeed();
			else
				EnableXFeed();
		}

		////////////////////////////////////////
		// IDockable

		private DockInfo dockInfo;

		public Part GetPart()
		{ return part; }

		public Transform GetNodeTransform()
		{ return nodeTransform; }

		public Vector3 GetDockingOrientation()
		{ return dockingOrientation; }

		public int GetSnapCount()
		{ return snapCount; }

		public DockInfo GetDockInfo()
		{ return dockInfo; }

		public void SetDockInfo(DockInfo _dockInfo)
		{
			dockInfo = _dockInfo;

			if(dockInfo == null)
			{
				dockedType = 0;
				vesselInfo = null;
			}
			else if(dockInfo.part == (IDockable)this)
			{
				dockedType = 0;
				vesselInfo = dockInfo.vesselInfo;
			}
			else
			{
				dockedType = 1;
				vesselInfo = dockInfo.targetVesselInfo;
			}
		}

		// returns true, if the port is compatible with the other port
		public bool IsCompatible(IDockable otherPort)
		{
			if(otherPort == null)
				return false;

			ModuleIRGF _otherPort = otherPort.GetPart().GetComponent<ModuleIRGF>();

			if(!_otherPort)
				return false;

			if(!nodeTypesAcceptedS.Contains(_otherPort.nodeType)
			|| !_otherPort.nodeTypesAcceptedS.Contains(nodeType))
				return false;

			return true;
		}

		public bool IsReadyFor(IDockable otherPort)
		{
			return false;
		}

		public ITargetable GetTargetable()
		{
			return null;
		}

		public bool IsDocked()
		{
			return ((fsm.CurrentState == st_docked) || (fsm.CurrentState == st_preattached));
		}

		public IDockable GetOtherDockable()
		{
			return IsDocked() ? (IDockable)otherPort : null;
		}

		////////////////////////////////////////
		// IModuleInfo

		string IModuleInfo.GetModuleTitle()
		{
			return "Latching End Effector";
		}

		string IModuleInfo.GetInfo()
		{
			string info = "";

			info += "AutoCapture: " + (canAutoCapture ? "<color=green>available</color>" : "<color=red>no</color>") + "\n";

			info += "Crossfeed: " + (crossfeed ? "<color=green>supported</color>" : "<color=red>no</color>") + "\n";

			if((electricChargeRequiredLatching > 0f) && (electricChargeRequiredReleasing > 0f))
				info += "\n<b><color=orange>Requires:</color></b>\n- <b>Electric Charge: </b>for latching and releasing";
			else if(electricChargeRequiredLatching > 0f) 
				info += "\n<b><color=orange>Requires:</color></b>\n- <b>Electric Charge: </b>for latching";
			else if(electricChargeRequiredReleasing > 0f)
				info += "\n<b><color=orange>Requires:</color></b>\n- <b>Electric Charge: </b>for releasing";

			return info;
		}

		Callback<Rect> IModuleInfo.GetDrawModulePanelCallback()
		{
			return null;
		}

		string IModuleInfo.GetPrimaryField()
		{
			return null;
		}

		////////////////////////////////////////
		// IResourceConsumer

		private List<PartResourceDefinition> consumedResources;

		public List<PartResourceDefinition> GetConsumedResources()
		{
			return consumedResources;
		}

		////////////////////////////////////////
		// IConstruction

		public bool CanBeDetached()
		{
			return fsm.CurrentState == st_disabled;
		}

		public bool CanBeOffset()
		{
			return fsm.CurrentState == st_disabled;
		}

		public bool CanBeRotated()
		{
			return fsm.CurrentState == st_disabled;
		}

		////////////////////////////////////////
		// Debug

#if DEBUG
	/*
		private Utility.MultiLineDrawer ld;

		private String[] astrDebug;
		private int istrDebugPos;

		private void DebugInit()
		{
			ld = new Utility.MultiLineDrawer();
			ld.Create(null);

			astrDebug = new String[10240];
			istrDebugPos = 0;
		}

		private void DebugString(String s)
		{
			astrDebug[istrDebugPos] = s;
			istrDebugPos = (istrDebugPos + 1) % 10240;
		}

		private void DrawPointer(int idx, Vector3 p_vector)
		{
			ld.Draw(idx, idx, Vector3.zero, p_vector);
		}

		private void DrawRelative(int idx, Vector3 p_from, Vector3 p_vector)
		{
			ld.Draw(idx, idx, p_from, p_from + p_vector);
		}

		private void DrawAxis(int idx, Transform p_transform, Vector3 p_vector, bool p_relative, Vector3 p_off)
		{
			ld.Draw(idx, idx, p_transform.position + p_off, p_transform.position + p_off
				+ (p_relative ? p_transform.TransformDirection(p_vector) : p_vector));
		}

		private void DrawAxis(int idx, Transform p_transform, Vector3 p_vector, bool p_relative)
		{ DrawAxis(idx, p_transform, p_vector, p_relative, Vector3.zero); }
	*/
#endif

	}
}
