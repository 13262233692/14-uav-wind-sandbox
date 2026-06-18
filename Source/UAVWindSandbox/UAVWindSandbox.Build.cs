using UnrealBuildTool;

public class UAVWindSandbox : ModuleRules
{
	public UAVWindSandbox(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"EnhancedInput",
			"ProceduralMeshComponent"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"RenderCore",
			"RHI"
		});

		PublicIncludePaths.AddRange(new string[]
		{
			"UAVWindSandbox"
		});
	}
}
