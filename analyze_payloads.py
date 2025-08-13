import csv
from collections import defaultdict

def calculate_weighted_median(weights, quantities):
    """Calculate weighted median given weights and their quantities"""
    # Create expanded list where each weight appears quantity times
    expanded_weights = []
    for weight, qty in zip(weights, quantities):
        expanded_weights.extend([weight] * qty)
    
    # Calculate median of expanded list
    expanded_weights.sort()
    n = len(expanded_weights)
    if n % 2 == 0:
        return (expanded_weights[n//2 - 1] + expanded_weights[n//2]) / 2
    else:
        return expanded_weights[n//2]

def analyze_payload_bins():
    """Analyze payload bins and calculate quantity-weighted medians"""
    payload_data = defaultdict(list)
    
    # Read the cleanparts.csv file
    with open('cleanparts.csv', 'r') as file:
        reader = csv.DictReader(file)
        
        for row in reader:
            payload_num = int(row['payload_number'])
            weight = float(row['weight_kg'])
            quantity = int(row['quantity'])
            
            payload_data[payload_num].append((weight, quantity))
    
    # Calculate statistics for each payload bin
    print("Payload Analysis (Quantity-Weighted):")
    print("=" * 50)
    
    for payload_num in sorted(payload_data.keys()):
        parts = payload_data[payload_num]
        weights = [part[0] for part in parts]
        quantities = [part[1] for part in parts]
        
        # Calculate weighted median
        weighted_median = calculate_weighted_median(weights, quantities)
        
        # Calculate weighted average
        total_weight = sum(weight * qty for weight, qty in zip(weights, quantities))
        total_quantity = sum(quantities)
        weighted_average = total_weight / total_quantity
        
        # Calculate other statistics
        total_parts = len(parts)
        min_weight = min(weights)
        max_weight = max(weights)
        weight_range = f"{min_weight:.3f}-{max_weight:.3f}kg"
        
        print(f"Payload {payload_num}: {weight_range}")
        print(f"  Weighted Average: {weighted_average:.3f} kg")
        print(f"  Weighted Median:  {weighted_median:.3f} kg")
        print(f"  Part Types: {total_parts}")
        print(f"  Total Quantity: {total_quantity:,}")
        print()

if __name__ == "__main__":
    analyze_payload_bins()