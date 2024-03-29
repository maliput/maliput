#include "maliput/drake/systems/framework/system_base.h"

#include <atomic>
#include <sstream>

#include "maliput/drake/systems/framework/fixed_input_port_value.h"

namespace {

// Output a string like "System::EvalInput()".
std::string FmtFunc(const char* func) {
  std::ostringstream oss;
  oss << "System::" << func << "()";
  return oss.str();
}

}  // namespace

namespace maliput::drake {
namespace systems {

SystemBase::~SystemBase() {}

internal::SystemId SystemBase::get_next_id() { return internal::SystemId::get_new_id(); }

std::string SystemBase::GetSystemPathname() const {
  const std::string parent_path = get_parent_service() ? get_parent_service()->GetParentPathname() : std::string();
  return parent_path + internal::SystemMessageInterface::path_separator() + GetSystemName();
}

CacheEntry& SystemBase::DeclareCacheEntry(std::string description, ValueProducer value_producer,
                                          std::set<DependencyTicket> prerequisites_of_calc) {
  return DeclareCacheEntryWithKnownTicket(assign_next_dependency_ticket(), std::move(description),
                                          std::move(value_producer), std::move(prerequisites_of_calc));
}

// (This overload is deprecated.)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
CacheEntry& SystemBase::DeclareCacheEntry(std::string description, CacheEntry::AllocCallback alloc_function,
                                          CacheEntry::CalcCallback calc_function,
                                          std::set<DependencyTicket> prerequisites_of_calc) {
  return DeclareCacheEntry(std::move(description), ValueProducer(std::move(alloc_function), std::move(calc_function)),
                           std::move(prerequisites_of_calc));
}
#pragma GCC diagnostic pop

CacheEntry& SystemBase::DeclareCacheEntryWithKnownTicket(DependencyTicket known_ticket, std::string description,
                                                         ValueProducer value_producer,
                                                         std::set<DependencyTicket> prerequisites_of_calc) {
  // If the prerequisite list is empty the CacheEntry constructor will throw
  // a logic error.
  const CacheIndex index(num_cache_entries());
  cache_entries_.emplace_back(std::make_unique<CacheEntry>(
      this, index, known_ticket, std::move(description), std::move(value_producer), std::move(prerequisites_of_calc)));
  CacheEntry& new_entry = *cache_entries_.back();
  return new_entry;
}

void SystemBase::InitializeContextBase(ContextBase* context_ptr) const {
  MALIPUT_DRAKE_DEMAND(context_ptr != nullptr);
  ContextBase& context = *context_ptr;

  // Initialization should happen only once per Context.
  MALIPUT_DRAKE_DEMAND(!internal::SystemBaseContextBaseAttorney::is_context_base_initialized(context));

  internal::SystemBaseContextBaseAttorney::set_system_name(&context, get_name());
  internal::SystemBaseContextBaseAttorney::set_system_id(&context, system_id_);

  // Add the independent-source trackers and wire them up appropriately. That
  // includes input ports since their dependencies are external.
  CreateSourceTrackers(&context);

  DependencyGraph& graph = context.get_mutable_dependency_graph();

  // Create the Context cache containing a CacheEntryValue corresponding to
  // each CacheEntry, add a DependencyTracker and subscribe it to its
  // prerequisites as specified in the CacheEntry. Cache entries are
  // necessarily ordered such that the first cache entry can depend only
  // on the known source trackers created above, the second may depend on
  // those plus the first, and so on. Circular dependencies are not permitted.
  Cache& cache = context.get_mutable_cache();
  for (CacheIndex index(0); index < num_cache_entries(); ++index) {
    const CacheEntry& entry = get_cache_entry(index);
    CacheEntryValue& cache_value = cache.CreateNewCacheEntryValue(entry.cache_index(), entry.ticket(),
                                                                  entry.description(), entry.prerequisites(), &graph);
    // TODO(sherm1) Supply initial value on creation instead and get rid of
    // this separate call.
    cache_value.SetInitialValue(entry.Allocate());

    if (entry.is_disabled_by_default()) cache_value.disable_caching();
  }

  // Create the output port trackers yᵢ here. Nothing in this System may
  // depend on them; subscribers will be input ports from peer subsystems or
  // an exported output port in the parent Diagram. The associated cache entries
  // were just created above. Any intra-system prerequisites are set up now.
  for (const auto& oport : output_ports_) {
    internal::SystemBaseContextBaseAttorney::AddOutputPort(&context, oport->get_index(), oport->ticket(),
                                                           oport->GetPrerequisite());
  }

  internal::SystemBaseContextBaseAttorney::mark_context_base_initialized(&context);
}

// Set up trackers for variable-numbered independent sources: discrete and
// abstract state, numerical and abstract parameters, and input ports.
// The generic trackers like "all parameters" are already present in the
// supplied Context, but we have to subscribe them to the individual
// elements now.
void SystemBase::CreateSourceTrackers(ContextBase* context_ptr) const {
  ContextBase& context = *context_ptr;

  // Define a lambda to do the repeated work below: create trackers for
  // individual entities and subscribe the group tracker to each of them.
  auto make_trackers = [&context](DependencyTicket subscriber_ticket,
                                  const std::vector<TrackerInfo>& system_ticket_info,
                                  void (*add_ticket_to_context)(ContextBase*, DependencyTicket)) {
    DependencyGraph& graph = context.get_mutable_dependency_graph();
    DependencyTracker& subscriber = graph.get_mutable_tracker(subscriber_ticket);

    for (const auto& info : system_ticket_info) {
      auto& source_tracker = graph.CreateNewDependencyTracker(info.ticket, info.description);
      add_ticket_to_context(&context, info.ticket);
      subscriber.SubscribeToPrerequisite(&source_tracker);
    }
  };

  // Allocate trackers for each discrete variable group xdᵢ, and subscribe
  // the "all discrete variables" tracker xd to those.
  make_trackers(xd_ticket(), discrete_state_tickets_, &internal::SystemBaseContextBaseAttorney::AddDiscreteStateTicket);

  // Allocate trackers for each abstract state variable xaᵢ, and subscribe
  // the "all abstract variables" tracker xa to those.
  make_trackers(xa_ticket(), abstract_state_tickets_, &internal::SystemBaseContextBaseAttorney::AddAbstractStateTicket);

  // Allocate trackers for each numeric parameter pnᵢ and each abstract
  // parameter paᵢ, and subscribe the pn and pa trackers to them.
  make_trackers(pn_ticket(), numeric_parameter_tickets_,
                &internal::SystemBaseContextBaseAttorney::AddNumericParameterTicket);
  make_trackers(pa_ticket(), abstract_parameter_tickets_,
                &internal::SystemBaseContextBaseAttorney::AddAbstractParameterTicket);

  // Allocate trackers for each input port uᵢ, and subscribe the "all input
  // ports" tracker u to them. Doesn't use TrackerInfo so can't use the lambda.
  for (const auto& iport : input_ports_) {
    internal::SystemBaseContextBaseAttorney::AddInputPort(&context, iport->get_index(), iport->ticket(),
                                                          MakeFixInputPortTypeChecker(iport->get_index()));
  }
}

// The only way for a system to evaluate its own input port is if that
// port is fixed. In that case the port's value is in the corresponding
// subcontext and we can just return it. Otherwise, the port obtains its value
// from some other system and we need our parent's help to get access to
// that system.
const AbstractValue* SystemBase::EvalAbstractInputImpl(const char* func, const ContextBase& context,
                                                       InputPortIndex port_index) const {
  if (port_index >= num_input_ports()) ThrowInputPortIndexOutOfRange(func, port_index);

  const FixedInputPortValue* const free_port_value = context.MaybeGetFixedInputPortValue(port_index);

  if (free_port_value != nullptr) return &free_port_value->get_value();  // A fixed input port.

  // The only way to satisfy an input port of a root System is to make it fixed.
  // Since it wasn't fixed, it is unconnected.
  if (get_parent_service() == nullptr) return nullptr;

  // If this is a root Context, our parent can't evaluate it.
  if (context.is_root_context()) return nullptr;

  // This is not the root System, and the port isn't fixed, so ask our parent to
  // evaluate it.
  return get_parent_service()->EvalConnectedSubsystemInputPort(
      *internal::SystemBaseContextBaseAttorney::get_parent_base(context), get_input_port_base(port_index));
}

void SystemBase::ThrowNegativePortIndex(const char* func, int port_index) const {
  MALIPUT_DRAKE_DEMAND(port_index < 0);
  std::ostringstream oss;
  oss << FmtFunc(func);
  oss << ": negative port index ";
  oss << port_index << " is illegal. (System ";
  oss << GetSystemPathname() << ")";
  throw std::out_of_range(oss.str());
}

void SystemBase::ThrowInputPortIndexOutOfRange(const char* func, InputPortIndex port) const {
  std::ostringstream oss;
  oss << FmtFunc(func);
  oss << ": there is no input port with index ";
  oss << port << " because there ";
  oss << "are only " << num_input_ports() << " input ports ";
  oss << "in system " << GetSystemPathname() << ".";

  throw std::out_of_range(oss.str());
}

void SystemBase::ThrowOutputPortIndexOutOfRange(const char* func, OutputPortIndex port) const {
  std::ostringstream oss;
  oss << FmtFunc(func);
  oss << ": there is no output port with index ";
  oss << port << " because there are only ";
  oss << num_output_ports() << " output ports in system ";
  oss << GetSystemPathname() << ".";
  throw std::out_of_range(oss.str());
}

void SystemBase::ThrowNotAVectorInputPort(const char* func, InputPortIndex port) const {
  std::ostringstream oss;
  oss << FmtFunc(func);
  oss << ": vector port required, but input port '";
  oss << get_input_port_base(port).get_name();
  oss << "' (index " << port << ") was declared ";
  oss << "abstract. Even if the actual value is a vector, use ";
  oss << "EvalInputValue<V> instead for an abstract port containing a vector ";
  oss << "of type V. (System " << GetSystemPathname() << ")";
  throw std::logic_error(oss.str());
}

void SystemBase::ThrowInputPortHasWrongType(const char* func, InputPortIndex port, const std::string& expected_type,
                                            const std::string& actual_type) const {
  ThrowInputPortHasWrongType(func, GetSystemPathname(), port, get_input_port_base(port).get_name(), expected_type,
                             actual_type);
}

void SystemBase::ThrowInputPortHasWrongType(const char* func, const std::string& system_pathname, InputPortIndex port,
                                            const std::string& port_name, const std::string& expected_type,
                                            const std::string& actual_type) {
  std::ostringstream oss;
  oss << FmtFunc(func) << ": expected value of type ";
  oss << expected_type;
  oss << " for input port '" << port_name << "'";
  oss << " (index " << port << ")";
  oss << " but the actual type was ";
  oss << actual_type << ". (System " << system_pathname << ")";
  throw std::logic_error(oss.str());
}

void SystemBase::ThrowCantEvaluateInputPort(const char* func, InputPortIndex port) const {
  std::ostringstream oss;
  oss << FmtFunc(func) << ": input port '";
  oss << get_input_port_base(port).get_name() << "' (index ";
  oss << port;
  oss << ") is neither connected nor fixed so cannot be evaluated.";
  oss << " (System " << GetSystemPathname() << ")";
  throw std::logic_error(oss.str());
}

void SystemBase::ThrowValidateContextMismatch(const ContextBase& context) const {
  std::stringstream oss;
  oss << "Context was not created for ";
  oss << this->GetSystemType();
  oss << " system ";
  oss << this->GetSystemPathname();
  oss << " it was created for system ";
  oss << context.GetSystemPathname();
  throw std::logic_error(oss.str());
}

void SystemBase::ThrowNotCreatedForThisSystemImpl(const std::string& nice_type_name, internal::SystemId id) const {
  if (!id.is_valid()) {
    std::stringstream oss;
    oss << nice_type_name;
    oss << " was not associated with any System but should have been ";
    oss << "created for ";
    oss << GetSystemType() << " System ";
    oss << GetSystemPathname();
    throw std::logic_error(oss.str());
  } else {
    std::stringstream oss;
    oss << nice_type_name << " was not created for ";
    oss << GetSystemType() << " System " << GetSystemPathname();
    throw std::logic_error(oss.str());
  }
}

[[noreturn]] void SystemBase::ThrowUnsupportedScalarConversion(const SystemBase& from,
                                                               const std::string& destination_type_name) {
  std::stringstream ss;
  ss << "The object named [" << from.get_name() << "] of type " << NiceTypeName::Get(from)
     << " does not support scalar conversion to type [" << destination_type_name << "].";
  throw std::logic_error(ss.str().c_str());
}

}  // namespace systems
}  // namespace maliput::drake
